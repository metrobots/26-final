package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
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
     * 0.228 m forward, 0.061 m left.
     */
    private static final Translation2d TURRET_PIVOT_FROM_ROBOT =
            new Translation2d(0.228, -0.061);

    /**
     * Vector from turret pivot to camera lens, expressed in turret-local frame.
     * Measure this physically — update as needed.
     * Example: camera is 0.1524 m (6 in) behind the pivot, centered laterally.
     */
        private static final Translation2d CAMERA_FROM_TURRET_PIVOT =
                new Translation2d(-0.147, 0.0);

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

        // The camera rotates with the turret, so MegaTag2 needs the camera's
        // actual field-space yaw (robot heading + turret offset), not just robot heading.
        Rotation2d turretHeadingField = getGyroRotation()
                .plus(Rotation2d.fromDegrees(turret.getTurretAngleRelative()));

        // Tell MegaTag2 the camera's true field-relative heading and robot yaw rate.
        // Using turretHeadingField here is critical — passing only gyro yaw would make
        // MegaTag2 think the whole robot rotated whenever the turret moves.
        LimelightLib.SetRobotOrientation(
                LIMELIGHT,
                turretHeadingField.getDegrees(),
                getTurnRate(),
                0, 0, 0, 0
        );

        PoseEstimate vision = LimelightLib.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);

        if (!LimelightLib.validPoseEstimate(vision)) return;

        // Reject noisy measurements during fast rotation (MegaTag2 degrades quickly here)
        if (Math.abs(getTurnRate()) > 720) return;

        // -----------------------------------------------------------------------
        // MegaTag2 reports the pose of whatever frame the Limelight camera
        // offset is set to in the web UI.  Since the camera is on a rotating
        // turret we do NOT use the web-UI offset; instead we treat the reported
        // translation as the camera lens position in field space and manually
        // walk back to robot center using live turret angle data.
        // -----------------------------------------------------------------------

        Translation2d cameraPositionField = vision.pose.getTranslation();

        // --- Step 1: turret heading in field space (already computed above) ---

        // --- Step 2: rotate each offset into field frame ---
        // Pivot offset is fixed in robot frame → rotate by robot heading
        Translation2d pivotOffsetField =
                TURRET_PIVOT_FROM_ROBOT.rotateBy(getGyroRotation());

        // Camera offset is fixed in turret frame → rotate by turret field heading
        Translation2d cameraOffsetField =
                CAMERA_FROM_TURRET_PIVOT.rotateBy(turretHeadingField);

        // --- Step 3: walk from camera lens → robot center ---
        // camera = robot + pivotOffset + cameraOffset
        // → robot = camera - cameraOffset - pivotOffset
        Translation2d robotPositionField = cameraPositionField
                .minus(cameraOffsetField)
                .minus(pivotOffsetField);

        // Use gyro for heading — never trust MegaTag2's rotation output
        Pose2d robotPose = new Pose2d(robotPositionField, getGyroRotation());

        poseEstimator.addVisionMeasurement(robotPose, vision.timestampSeconds);

        // Debug: show raw camera position on field
        field.getObject("VisionCamera").setPose(
                new Pose2d(cameraPositionField, turretHeadingField));
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

    public double getTurnRate()       { return -gyro.getRate(); }
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