package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.PoseEstimate;

public class Drivetrain extends SubsystemBase {

    private final Turret turret;
    private static final String LIMELIGHT = "limelight-front";

    /* ================= FIELD CONSTANTS ================= */

    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH  = 8.21;

    private static final Translation2d FIELD_CENTER =
            new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

    /* ================= TURRET / CAMERA GEOMETRY ================= */

    /**
     * Vector from robot center to turret pivot, expressed in robot frame.
     * +X = forward, +Y = left.
     */
    private static final double TURRET_PIVOT_FORWARD = 0.228;  // meters
    private static final double TURRET_PIVOT_SIDE    = -0.061; // meters (negative = right)

    /**
     * Distance from turret pivot to camera lens along the turret's forward axis.
     * Negative because camera is behind the pivot.
     */
    private static final double CAMERA_FROM_PIVOT_AXIAL = -0.147; // meters

    /**
     * Camera height from ground: 21 inches converted to meters.
     */
    private static final double CAMERA_HEIGHT = 21.0 * 0.0254; // 0.5334 meters

    /**
     * Camera pitch: 30 degrees upward from horizontal.
     * Negative sign because Limelight uses negative pitch for upward tilt.
     */
    private static final double CAMERA_PITCH = -30.0; // degrees

    /* ================= FIELD DISPLAY ================= */

    private final Field2d field = new Field2d();

    /* ================= STATE ================= */

    private double angleToCenter    = 0;
    private double distanceToCenter = 0;

    /* ================= MODULES ================= */

    private final Module frontLeft = new Module(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftEncoder,
            DriveConstants.kFrontLeftEncoderOffset);

    private final Module frontRight = new Module(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightEncoder,
            DriveConstants.kFrontRightEncoderOffset);

    private final Module rearLeft = new Module(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kRearLeftEncoder,
            DriveConstants.kRearLeftEncoderOffset);

    private final Module rearRight = new Module(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kRearRightEncoder,
            DriveConstants.kRearRightEncoderOffset);

    /* ================= SENSORS ================= */

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    /* ================= CONTROL ================= */

    private final PIDController headingCorrector = new PIDController(0, 0, 0);

    private final SlewRateLimiter xLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final SlewRateLimiter yLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final SlewRateLimiter rotLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    /* ================= POSE ESTIMATOR ================= */

    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.kDriveKinematics,
                    getGyroRotation(),
                    getModulePositions(),
                    new Pose2d()
            );

    /* ================= CONSTRUCTOR ================= */

    public Drivetrain(Turret turret) {
        this.turret = turret;
        SmartDashboard.putData("Field", field);
        configureAutoBuilder();
    }

    /* ================= AUTO BUILDER ================= */

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotRelativeSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0),
                            new PIDConstants(5, 0, 0)
                    ),
                    config,
                    () -> DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                    this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
        }
    }

    /* ================= PERIODIC ================= */

    @Override
    public void periodic() {
        updateOdometry();
        updateVision();
        updateFieldCalculations();
        updateDashboard();
    }

    /* ================= ODOMETRY ================= */

    private void updateOdometry() {
        poseEstimator.update(getGyroRotation(), getModulePositions());
    }

    /* ================= VISION ================= */

    private void updateVision() {

        double turretAngleDeg = turret.getTurretAngleRelative();
        double turretAngleRad = Math.toRadians(turretAngleDeg);

        // Compute the camera's current position in robot space by rotating
        // the axial camera offset (along the turret's forward axis) by the
        // live turret angle, then adding the fixed pivot offset from robot center.
        double camForward = TURRET_PIVOT_FORWARD + (CAMERA_FROM_PIVOT_AXIAL * Math.cos(turretAngleRad));
        double camSide    = TURRET_PIVOT_SIDE    + (CAMERA_FROM_PIVOT_AXIAL * Math.sin(turretAngleRad));

        // Tell Limelight exactly where the camera is in robot space right now.
        // Per the docs, call this BEFORE SetRobotOrientation so both values
        // are flushed to the Limelight in the same update.
        //
        // NOTE: The Limelight web UI camera pose must be set to all zeros —
        // this call fully replaces the static web UI offset each loop.
        LimelightLib.setCameraPose_RobotSpace(
                LIMELIGHT,
                camForward,      // forward from robot center (meters)
                camSide,         // left of robot center (meters, negative = right)
                CAMERA_HEIGHT,   // up from robot center (meters)
                0.0,             // roll (degrees)
                CAMERA_PITCH,    // pitch (degrees, negative = upward tilt)
                turretAngleDeg   // yaw (degrees, live turret angle relative to robot forward)
        );

        // Pass only the gyro heading — camera yaw is already handled above
        // by setCameraPose_RobotSpace, so we do NOT pass turretHeadingField here.
        LimelightLib.SetRobotOrientation(
                LIMELIGHT,
                getGyroRotation().getDegrees(),
                getTurnRate(),
                0, 0, 0, 0
        );

        PoseEstimate vision = LimelightLib.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);

        if (!LimelightLib.validPoseEstimate(vision)) return;

        // Reject noisy measurements during fast rotation
        if (Math.abs(getTurnRate()) > 720) return;

        // MegaTag2 now has full knowledge of camera position and heading via
        // setCameraPose_RobotSpace, so its output is already robot-center pose.
        // Always override rotation with gyro — never trust MegaTag2 for heading.
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
        poseEstimator.addVisionMeasurement(
                new Pose2d(vision.pose.getTranslation(), getGyroRotation()),
                vision.timestampSeconds
        );

        // Debug: show MegaTag2's raw pose output on the field display
        field.getObject("VisionPose").setPose(vision.pose);
    }

    /* ================= FIELD CALCULATIONS ================= */

    private void updateFieldCalculations() {

        Pose2d pose  = poseEstimator.getEstimatedPosition();
        field.setRobotPose(pose);

        Translation2d robot = pose.getTranslation();
        distanceToCenter = robot.getDistance(FIELD_CENTER);

        double dx = FIELD_CENTER.getX() - robot.getX();
        double dy = FIELD_CENTER.getY() - robot.getY();

        double fieldAngle   = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = getGyroRotation().getDegrees();

        angleToCenter = MathUtil.inputModulus(fieldAngle - robotHeading, -180, 180);
    }

    /* ================= DASHBOARD ================= */

    private void updateDashboard() {
        SmartDashboard.putNumber("Robot Heading",      getGyroRotation().getDegrees());
        SmartDashboard.putNumber("Angle To Center",    angleToCenter);
        SmartDashboard.putNumber("Distance To Center", distanceToCenter);
    }

    /* ================= POSE METHODS ================= */

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public double getAngleToCenter()    { return angleToCenter; }
    public double getDistanceToCenter() { return distanceToCenter; }

    /* ================= DRIVE ================= */

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        double x = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
        double y = yLimiter.calculate(ySpeed)  * DriveConstants.kMaxSpeedMetersPerSecond;
        double r = rotLimiter.calculate(rot)   * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, getGyroRotation())
                : new ChassisSpeeds(x, y, r);

        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void driveLocked(double xSpeed, double ySpeed, double targetHeading, boolean fieldRelative) {
        double correction = headingCorrector.calculate(getHeading(), targetHeading);
        drive(xSpeed, ySpeed, correction, fieldRelative);
    }

    /* ================= MODULE CONTROL ================= */

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        rearLeft.setDesiredState(states[2]);
        rearRight.setDesiredState(states[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState()
        );
    }

    /* ================= GYRO ================= */

    public void zeroHeading() { gyro.reset(); }

    public double getHeading() {
        return MathUtil.inputModulus(-gyro.getAngle(), -180, 180);
    }

    public double getTurnRate()         { return -gyro.getRate(); }
    public Rotation2d getGyroRotation() { return Rotation2d.fromDegrees(-gyro.getAngle()); }

    /* ================= UTIL ================= */

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }
}