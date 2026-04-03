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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Elastic;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.PoseEstimate;

import java.util.List;

public class Drivetrain extends SubsystemBase {

    private final Turret turret;
    private static final String LIMELIGHT = "limelight-front";
    private boolean endgame = false;

    /* ================= HUB STATUS ================= */
    private boolean hubActive          = true;
    private String  hubStatus          = "Unknown";
    private int     currentShift       = 0;
    private double  shiftTimeRemaining = 0;

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

    /* ================= FIELD DISPLAY ================= */
    private final Field2d field = new Field2d();

    /* ================= STATE ================= */
    // angleToCenter  : angle from robot forward to target, in robot frame, –180..180 deg.
    //                  Positive = target is to the robot's left (CCW).
    // distanceToCenter: straight-line distance from turret pivot to target, metres.
    private double angleToCenter    = 0;
    private double distanceToCenter = 0;
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

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
    private final AHRS gyro = new AHRS(NavXComType.kUSB1);

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
            new Pose2d(0, 0, getGyroRotation())
        );

    /* ================= CONSTRUCTOR ================= */
    public Drivetrain(Turret turret) {
        this.turret = turret;
        SmartDashboard.putData("Field", field);
        endgame = false;
        configureAutoBuilder();

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Front Left Angle",     () -> frontLeft.getState().angle.getRadians(),    null);
                builder.addDoubleProperty("Front Left Velocity",  () -> frontLeft.getState().speedMetersPerSecond,   null);
                builder.addDoubleProperty("Front Right Angle",    () -> frontRight.getState().angle.getRadians(),   null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond,  null);
                builder.addDoubleProperty("Back Left Angle",      () -> rearLeft.getState().angle.getRadians(),     null);
                builder.addDoubleProperty("Back Left Velocity",   () -> rearLeft.getState().speedMetersPerSecond,    null);
                builder.addDoubleProperty("Back Right Angle",     () -> rearRight.getState().angle.getRadians(),    null);
                builder.addDoubleProperty("Back Right Velocity",  () -> rearRight.getState().speedMetersPerSecond,  null);
                builder.addDoubleProperty("Robot Angle",          () -> getGyroRotation().getRadians(),              null);
            }
        });
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
        updateFieldRelativeSpeeds();
        updateVision();
        updateFieldCalculations();
        updateHubStatus();
        updateDashboard();

        if (DriverStation.getMatchTime() < 20 && !endgame) {
            Elastic.selectTab(2);
            endgame = true;
        }
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

    /* ================= FIELD CALCULATIONS ================= */

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

    /* ================= HUB STATUS ================= */
    private void updateHubStatus() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            hubActive = false;
            hubStatus = "Unknown";
            return;
        }

        if (DriverStation.isAutonomousEnabled()) {
            hubActive = true;
            hubStatus = "Active (Auto)";
            currentShift = 0;
            shiftTimeRemaining = DriverStation.getMatchTime();
            return;
        }

        if (!DriverStation.isTeleopEnabled()) {
            hubActive = false;
            hubStatus = "Inactive";
            currentShift = 0;
            shiftTimeRemaining = 0;
            return;
        }

        double t = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            hubActive = true;
            hubStatus = "Active";
            return;
        }

        boolean redInactiveFirst = gameData.charAt(0) == 'R';
        boolean isRed = alliance.get() == DriverStation.Alliance.Red;
        boolean shift1Active = isRed ? !redInactiveFirst : redInactiveFirst;

        if (t > 130) {
            hubActive = true;
            currentShift = 0;
            shiftTimeRemaining = t - 130;
        } else if (t > 105) {
            hubActive = shift1Active;
            currentShift = 1;
            shiftTimeRemaining = t - 105;
        } else if (t > 80) {
            hubActive = !shift1Active;
            currentShift = 2;
            shiftTimeRemaining = t - 80;
        } else if (t > 55) {
            hubActive = shift1Active;
            currentShift = 3;
            shiftTimeRemaining = t - 55;
        } else if (t > 30) {
            hubActive = !shift1Active;
            currentShift = 4;
            shiftTimeRemaining = t - 30;
        } else {
            hubActive = true;
            currentShift = 5;
            shiftTimeRemaining = t;
        }

        hubStatus = hubActive ? "Active" : "Inactive";
    }

    /* ================= DASHBOARD ================= */
    private void updateDashboard() {
        SmartDashboard.putNumber("Robot Heading",        getHeading());
        SmartDashboard.putNumber("Angle To Center",      angleToCenter);
        SmartDashboard.putNumber("Distance To Center",   distanceToCenter);
        SmartDashboard.putNumber("Lateral Velocity",     getLateralVelocityToTarget());
        SmartDashboard.putNumber("RR",                   rearRight.getAngle());
        SmartDashboard.putNumber("RL",                   rearLeft.getAngle());
        SmartDashboard.putNumber("RRoffset",             rearRight.getAngleFull());
        SmartDashboard.putNumber("RLoffset",             rearLeft.getAngleFull());
        SmartDashboard.putBoolean("Hub Active",          hubActive);
        SmartDashboard.putString("Hub Status",           hubStatus);
        SmartDashboard.putNumber("Hub Shift",            currentShift);
        SmartDashboard.putNumber("Shift Time Remaining", shiftTimeRemaining);
    }

    /* ================= POSE METHODS ================= */
    public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public double getAngleToCenter()    { return angleToCenter; }
    public double getDistanceToCenter() { return distanceToCenter; }

    /* ================= DRIVE ================= */
    /**
     * Drive the robot.
     *
     * xSpeed / ySpeed are in –1..1 (driver joystick scale).
     * rot is in –1..1.
     * fieldRelative=true → driver-centric; the robot drives in the field frame
     * regardless of which wall it started from.
     *
     * getGyroRotation() returns a heading in the WPILib blue-origin field frame,
     * which is what ChassisSpeeds.fromFieldRelativeSpeeds() expects.  No alliance
     * inversion is required here: on red the robot starts at 180°, so field-relative
     * driving already points the correct way without any extra offset.
     */
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

    /**
     * Field-relative drive with closed-loop heading hold.
     * targetHeading is in degrees in the WPILib field frame (–180..180).
     */
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
    public void zeroHeading()        { gyro.reset(); }
    public double getHeading()       { return MathUtil.inputModulus(getGyroRotation().getDegrees(), -180, 180); }
    public double getTurnRate()      { return -gyro.getRate(); }

    /**
     * Returns the robot heading in the WPILib blue-origin field frame.
     * NavX reports CW-positive; WPILib expects CCW-positive, so we negate.
     * This is consistent for both alliances — on red the robot simply starts
     * near 180° rather than near 0°.  No extra flip is applied.
     */
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