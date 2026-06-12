package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.PoseEstimate;

import java.util.List;

public class Drivetrain extends SubsystemBase {
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

    /* Used for limiting the acceleration of the robot. Takes in velocity (m/s) and outputs velocity (m/s). */
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    /* ================= FIELD DISPLAY ================= */
    private final Field2d field = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getGyroRotation(),
            getModulePositions(),
            new Pose2d(0, 0, getGyroRotation()));

    /* ================= SENSORS ================= */
    private final AHRS gyro = new AHRS(NavXComType.kUSB1);

    public Drivetrain(Turret turret) {
        this.turret = turret;
        configureAutoBuilder();
        sendSendablesToDashboard();
        zeroHeading();
    }

    /**
     * Sends `Sendable` objects to dashboard. This method should only be called once during initialization.
     */
    private void sendSendablesToDashboard() {
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond,
                        null);
                builder.addDoubleProperty("Back Left Angle", () -> rearLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> rearLeft.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Back Right Angle", () -> rearRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> rearRight.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Robot Angle", () -> getGyroRotation().getRadians(), null);
            }
        });
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left Raw Angle", frontLeft.getRawAngle());
        SmartDashboard.putNumber("Front Right Raw Angle", frontRight.getRawAngle());
        SmartDashboard.putNumber("Back Left Raw Angle", rearLeft.getRawAngle());
        SmartDashboard.putNumber("Back Right Raw Angle", rearRight.getRawAngle());
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
                            new PIDConstants(5, 0, 0)),
                    config,
                    () -> DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
        }
    }

    /* ================= PERIODIC ================= */
    @Override
    public void periodic() {
        updateOdometry();
        updateFieldRelativeSpeeds();
        updateDashboard();
    }

    /* ================= ODOMETRY ================= */
    private void updateOdometry() {
        poseEstimator.update(getGyroRotation(), getModulePositions());
    }

    /* ================= FIELD-RELATIVE SPEEDS ================= */
    private void updateFieldRelativeSpeeds() {
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(), getGyroRotation());
    }

    /* ================= POSE METHODS ================= */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    /* ================= DRIVE ================= */
    /**
     * Drive the robot using inputs from a gamepad. All of these these directions
     * are relative to a person standing at driver station.
     * TODO: Check signs and axes.
     * 
     * @param xInput          Speed of the robot in the x-direction (back-,
     *                        forward+). Range: [-1.0, 1.0].
     * @param yInput          Speed of the robot in the y-direction (right-, left+).
     *                        Range: [-1.0, 1.0].
     * @param rotInput        Rotational speed of the robot (CW-, CCW+). Range:
     *                        [-1.0, 1.0]
     * @param isFieldRelative Controls whether or not to drive relative to the field
     *                        (and driver station) or the robot.
     */
    public void drive(double xInput, double yInput, double rotInput, boolean isFieldRelative) {
        double xVel = xLimiter.calculate(xInput * DriveConstants.kMaxSpeedMetersPerSecond);
        double yVel = yLimiter.calculate(yInput * DriveConstants.kMaxSpeedMetersPerSecond);
        double rVel = rotLimiter.calculate(rotInput * DriveConstants.kMaxAngularSpeed);

        ChassisSpeeds speeds = isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rVel, getGyroRotation())
                : new ChassisSpeeds(xVel, yVel, rVel);

        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

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
                rearRight.getState());
    }

    /* ================= GYRO ================= */
    
    /**
     * Sets gyro heading to 0.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * @return The current heading of the robot in degrees.
     */
    public double getHeading() {
        return MathUtil.inputModulus(getGyroRotation().getDegrees(), -180, 180);
    }

    /**
     * @return The current turn rate of the robot in degrees per second.
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
    */
    public Rotation2d getGyroRotation() {
    // The NavX gyro uses a CCW- sign convention, but WPILIB expects CCW+ convention.
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /* ================= UTIL ================= */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
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

    /* ================= EXTERNAL ================= */
    /* All of the code below is used by other subsystems. In the future,
    it should be decoupled from drivetrain into those subsystems. */

    private final Turret turret;
    private static final String LIMELIGHT = "limelight-front";

  /* ================= FIELD CONSTANTS ================= */
    // All coordinates are in the WPILib blue-origin field frame (x=0 is blue wall).
    // Red coordinates are derived by mirroring: redX = FIELD_LENGTH - blueX.
    private static final double FIELD_LENGTH = 16.54;   // metres
    private static final double FIELD_WIDTH  =  8.21;   // metres

    // Target point (e.g. hub / scoring zone centre) in BLUE coordinates.
    // Mirror automatically for red — do NOT hard-code a separate red value.
    private static final double TARGET_BLUE_X = (FIELD_LENGTH / 3.0) - 1.0;
    private static final double TARGET_Y       = FIELD_WIDTH / 2.0;   // same for both alliances

    /* ================= TURRET / CAMERA GEOMETRY ================= */
    // All values are in the robot frame: +x = forward, +y = left.
    private static final double TURRET_PIVOT_FORWARD    =  0.228;   // m forward of robot centre
    private static final double TURRET_PIVOT_SIDE       =  0.061;   // m left   of robot centre
    private static final double CAMERA_FROM_PIVOT_AXIAL = -0.147;   // m axial along turret barrel
    private static final double CAMERA_HEIGHT            = 21.0 * 0.0254;  // m above ground
    private static final double CAMERA_PITCH             = 30.632901;      // degrees

    private double angleToCenter    = 0;
    private double distanceToCenter = 0;
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    /* ================= VISION ================= */
    /**
     * Updates the camera pose in Limelight's robot-space whenever the turret
     * rotates, then feeds MegaTag2 pose estimates into the pose estimator.
     *
     * Key alliance-correctness notes:
     *  - setCameraPose_RobotSpace uses robot-frame geometry only; no alliance flip needed.
     *  - SetRobotOrientation must receive the raw gyro heading in the WPILib blue-origin
     *    field frame.  getGyroRotation() already returns that (NavX negated), so we pass
     *    it directly — no 180° flip for red.  The 180° flip was wrong: it fed Limelight
     *    a heading in a different frame than MegaTag2 expects, corrupting MT2 on red.
     *  - We override the vision pose's rotation with the gyro rotation before fusing, so
     *    the estimator only absorbs translational (x/y) corrections from the camera.
     */
    private void updateVision() {
        double turretAngleDeg = turret.getTurretAngleRelative();
        double turretAngleRad = Math.toRadians(turretAngleDeg);

        // Camera position in robot space — purely geometric, alliance-independent.
        double camForward = TURRET_PIVOT_FORWARD + CAMERA_FROM_PIVOT_AXIAL * Math.cos(turretAngleRad);
        double camSide    = TURRET_PIVOT_SIDE    - CAMERA_FROM_PIVOT_AXIAL * Math.sin(turretAngleRad);

        LimelightLib.setCameraPose_RobotSpace(
                LIMELIGHT,
                camForward,
                camSide,
                CAMERA_HEIGHT,
                0.0,
                CAMERA_PITCH,
                turretAngleDeg
        );

        // Pass the raw WPILib-frame gyro heading.  MegaTag2 works in the same blue-origin
        // field frame as the pose estimator, so no alliance offset is applied here.
        LimelightLib.SetRobotOrientation(
                LIMELIGHT,
                getGyroRotation().getDegrees(),
                getTurnRate(),
                0, 0, 0, 0
        );

        PoseEstimate vision = LimelightLib.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);
        if (!LimelightLib.validPoseEstimate(vision)) return;
        if (Math.abs(getTurnRate()) > 720) return;

        // Fuse only the translational component of the vision estimate; keep the gyro
        // rotation so heading drift can't be introduced by the camera.
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
        poseEstimator.addVisionMeasurement(
                new Pose2d(vision.pose.getTranslation(), getGyroRotation()),
                vision.timestampSeconds
        );

        field.getObject("VisionPose").setPose(vision.pose);
    }
    
    /**
     * Returns the target position in the WPILib blue-origin field frame.
     * For red alliance the target is mirrored across the field's centre line (x-axis).
     * Y is the same for both alliances because the target sits at field midwidth.
     */
    private Translation2d getTargetPosition() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        double targetX = isRed ? FIELD_LENGTH - TARGET_BLUE_X : TARGET_BLUE_X;
        return new Translation2d(targetX, TARGET_Y);
    }
    
    /**
     * Recomputes angleToCenter and distanceToCenter from the current estimated pose.
     *
     * angleToCenter is the angle from the robot's forward direction to the target,
     * measured in the robot frame (degrees, –180..180, positive = left/CCW).
     * This is calculated as:
     *
     *   angleToCenter = wrapTo180( fieldAngleToTarget − robotHeading )
     *
     * where fieldAngleToTarget = atan2(dy, dx) in the blue-origin field frame.
     *
     * No alliance offset is needed because:
     *   - getTargetPosition() already returns the correct mirrored position for red.
     *   - getGyroRotation() already returns heading in the blue-origin field frame.
     * Both sides of the subtraction are in the same frame, so the result is
     * alliance-agnostic automatically.
     */
    private void updateFieldCalculations() {
        Pose2d pose = poseEstimator.getEstimatedPosition();
        field.setRobotPose(pose);

        double robotHeading = getGyroRotation().getRadians();

        // Compute turret pivot world position.
        double turretWorldX = pose.getX()
                + TURRET_PIVOT_FORWARD * Math.cos(robotHeading)
                - TURRET_PIVOT_SIDE   * Math.sin(robotHeading);
        double turretWorldY = pose.getY()
                + TURRET_PIVOT_FORWARD * Math.sin(robotHeading)
                + TURRET_PIVOT_SIDE   * Math.cos(robotHeading);

        Translation2d turretPivot = new Translation2d(turretWorldX, turretWorldY);
        Translation2d target      = getTargetPosition();

        distanceToCenter = turretPivot.getDistance(target);

        double dx = target.getX() - turretPivot.getX();
        double dy = target.getY() - turretPivot.getY();

        // Field-frame angle from turret pivot to target.
        double fieldAngleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Robot-frame angle to target = field angle to target − robot heading.
        // Wrapped to –180..180 so the turret always takes the short path.
        // No alliance offset — both getGyroRotation() and fieldAngleToTargetDeg are
        // already in the blue-origin field frame.
        angleToCenter = MathUtil.inputModulus(
                fieldAngleToTargetDeg - getGyroRotation().getDegrees(), -180, 180);

        field.getObject("Target").setPose(new Pose2d(target, new Rotation2d()));
        field.getObject("TurretPivot").setPose(new Pose2d(turretPivot, pose.getRotation()));
        field.getObject("ToTarget").setPoses(List.of(
                new Pose2d(turretPivot, new Rotation2d()),
                new Pose2d(target, new Rotation2d())));
    }

    public double getAngleToCenter()    { return angleToCenter; }
    public double getDistanceToCenter() { return distanceToCenter; }

    
    /* ================= SHOOT-ON-THE-MOVE ================= */
    /**
     * Returns the component of the robot's field-relative velocity that is
     * perpendicular to the line between the turret pivot and the target.
     * Sign convention: positive = robot moving to the left of that line.
     *
     * Uses angleToCenter (robot-frame angle to target) plus the robot heading
     * to build the field-frame unit vector along the turret-to-target line,
     * then dots the perpendicular of that vector against field-relative velocity.
     */
    public double getLateralVelocityToTarget() {
        // Direction from robot to target in the field frame (radians).
        double toTargetFieldRad = getGyroRotation().getRadians()
                - Math.toRadians(angleToCenter);

        // Perpendicular direction (90° CCW from the target direction).
        double perpX = -Math.sin(toTargetFieldRad);
        double perpY =  Math.cos(toTargetFieldRad);

        return fieldRelativeSpeeds.vxMetersPerSecond * perpX
             + fieldRelativeSpeeds.vyMetersPerSecond * perpY;
    }
}