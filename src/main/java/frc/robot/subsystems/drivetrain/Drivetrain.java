// package frc.robot.subsystems.drivetrain;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.math.kinematics.*;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.*;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.subsystems.turret.Turret;
// import frc.robot.utils.Constants.AutoConstants;
// import frc.robot.utils.Constants.DriveConstants;
// import frc.robot.utils.Elastic;
// import frc.robot.utils.LimelightLib;
// import frc.robot.utils.LimelightLib.PoseEstimate;

// import java.util.List;

// public class Drivetrain extends SubsystemBase {

//     private final Turret turret;
//     private static final String LIMELIGHT = "limelight-front";
//     private boolean endgame = false;

//     /* ================= HUB STATUS ================= */

//     private boolean hubActive          = true;
//     private String  hubStatus          = "Unknown";
//     private int     currentShift       = 0;
//     private double  shiftTimeRemaining = 0;

//     /* ================= FIELD CONSTANTS ================= */

//     private static final double FIELD_LENGTH = 16.54;
//     private static final double FIELD_WIDTH  = 8.21;

//     /** Base X for the red-side field center target. Mirrored for blue alliance. */
//     private static final double FIELD_CENTER_X = (FIELD_LENGTH / 3.0) - 1;

//     /* ================= TURRET / CAMERA GEOMETRY ================= */

//     /**
//      * Vector from robot center to turret pivot, expressed in robot frame.
//      * +X = forward, +Y = left.
//      */
//     private static final double TURRET_PIVOT_FORWARD    =  0.228;  // meters
//     private static final double TURRET_PIVOT_SIDE       =  0.061;  // meters (negative = right)

//     /**
//      * Distance from turret pivot to camera lens along the turret's forward axis.
//      * Negative because camera is behind the pivot.
//      */
//     private static final double CAMERA_FROM_PIVOT_AXIAL = -0.147;  // meters

//     /** Camera height from ground: 21 inches converted to meters. */
//     private static final double CAMERA_HEIGHT            = 21.0 * 0.0254; // 0.5334 m

//     /**
//      * Camera pitch: upward from horizontal.
//      * Negative sign because Limelight uses negative pitch for upward tilt.
//      */
//     private static final double CAMERA_PITCH             = 30.632901; // degrees

//     /* ================= FIELD DISPLAY ================= */

//     private final Field2d field = new Field2d();

//     /* ================= STATE ================= */

//     private double angleToCenter    = 0;
//     private double distanceToCenter = 0;

//     // Cached field-relative chassis speeds, updated every loop.
//     // Used by getLateralVelocityToTarget() for shoot-on-the-move lead compensation.
//     private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

//     /* ================= MODULES ================= */

//     private final Module frontLeft = new Module(
//             DriveConstants.kFrontLeftDrivingCanId,
//             DriveConstants.kFrontLeftTurningCanId,
//             DriveConstants.kFrontLeftEncoder,
//             DriveConstants.kFrontLeftEncoderOffset);

//     private final Module frontRight = new Module(
//             DriveConstants.kFrontRightDrivingCanId,
//             DriveConstants.kFrontRightTurningCanId,
//             DriveConstants.kFrontRightEncoder,
//             DriveConstants.kFrontRightEncoderOffset);

//     private final Module rearLeft = new Module(
//             DriveConstants.kRearLeftDrivingCanId,
//             DriveConstants.kRearLeftTurningCanId,
//             DriveConstants.kRearLeftEncoder,
//             DriveConstants.kRearLeftEncoderOffset);

//     private final Module rearRight = new Module(
//             DriveConstants.kRearRightDrivingCanId,
//             DriveConstants.kRearRightTurningCanId,
//             DriveConstants.kRearRightEncoder,
//             DriveConstants.kRearRightEncoderOffset);

//     /* ================= SENSORS ================= */

//     private final AHRS gyro = new AHRS(NavXComType.kUSB1);

//     /* ================= CONTROL ================= */

//     private final PIDController headingCorrector = new PIDController(0, 0, 0);

//     private final SlewRateLimiter xLimiter =
//             new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

//     private final SlewRateLimiter yLimiter =
//             new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

//     private final SlewRateLimiter rotLimiter =
//             new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

//     /* ================= POSE ESTIMATOR ================= */

//     private final SwerveDrivePoseEstimator poseEstimator =
//             new SwerveDrivePoseEstimator(
//                     DriveConstants.kDriveKinematics,
//                     getGyroRotation(),
//                     getModulePositions(),
//                     new Pose2d()
//             );

//     /* ================= CONSTRUCTOR ================= */

//     public Drivetrain(Turret turret) {
//         this.turret = turret;
//         SmartDashboard.putData("Field", field);
//         endgame = false;
//         configureAutoBuilder();

//         SmartDashboard.putData("Swerve Drive", new Sendable() {
//             @Override
//             public void initSendable(SendableBuilder builder) {
//                 builder.setSmartDashboardType("SwerveDrive");
//                 builder.addDoubleProperty("Front Left Angle",     () -> frontLeft.getState().angle.getRadians(),    null);
//                 builder.addDoubleProperty("Front Left Velocity",  () -> frontLeft.getState().speedMetersPerSecond,   null);
//                 builder.addDoubleProperty("Front Right Angle",    () -> frontRight.getState().angle.getRadians(),   null);
//                 builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond,  null);
//                 builder.addDoubleProperty("Back Left Angle",      () -> rearLeft.getState().angle.getRadians(),     null);
//                 builder.addDoubleProperty("Back Left Velocity",   () -> rearLeft.getState().speedMetersPerSecond,    null);
//                 builder.addDoubleProperty("Back Right Angle",     () -> rearRight.getState().angle.getRadians(),    null);
//                 builder.addDoubleProperty("Back Right Velocity",  () -> rearRight.getState().speedMetersPerSecond,  null);
//                 builder.addDoubleProperty("Robot Angle",          () -> getGyroRotation().getRadians(),              null);
//             }
//         });
//     }

//     /* ================= AUTO BUILDER ================= */

//     private void configureAutoBuilder() {
//         try {
//             RobotConfig config = RobotConfig.fromGUISettings();
//             AutoBuilder.configure(
//                     this::getPose,
//                     this::resetOdometry,
//                     this::getRobotRelativeSpeeds,
//                     this::driveRobotRelative,
//                     new PPHolonomicDriveController(
//                             new PIDConstants(5, 0, 0),
//                             new PIDConstants(5, 0, 0)
//                     ),
//                     config,
//                     () -> DriverStation.getAlliance().isPresent()
//                             && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
//                     this
//             );
//         } catch (Exception e) {
//             DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
//         }
//     }

//     /* ================= PERIODIC ================= */

//     @Override
//     public void periodic() {
//         updateOdometry();
//         updateFieldRelativeSpeeds();
//         updateVision();
//         updateFieldCalculations();
//         updateHubStatus();
//         updateDashboard();

//         if (DriverStation.getMatchTime() < 20 && !endgame) {
//             Elastic.selectTab(2);
//             endgame = true;
//         }
//     }

//     /* ================= ODOMETRY ================= */

//     private void updateOdometry() {
//         poseEstimator.update(getGyroRotation(), getModulePositions());
//     }

//     /* ================= FIELD-RELATIVE SPEEDS ================= */

//     /**
//      * Converts robot-relative module states into field-relative chassis speeds
//      * and caches them for use by getLateralVelocityToTarget().
//      */
//     private void updateFieldRelativeSpeeds() {
//         ChassisSpeeds robotRelative = getRobotRelativeSpeeds();
//         fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//                 robotRelative, getGyroRotation());
//     }

//     /* ================= SHOOT-ON-THE-MOVE ================= */

//     /**
//      * Returns the robot's lateral velocity (m/s) relative to the target direction.
//      *
//      * "Lateral" means perpendicular to the straight line from the turret pivot
//      * to the field center. Positive = moving right relative to that line
//      * (i.e., the note needs to lead left, so the turret swings CCW).
//      */
//     public double getLateralVelocityToTarget() {
//         double angleRad = Math.toRadians(angleToCenter);
//         return -fieldRelativeSpeeds.vxMetersPerSecond * Math.sin(angleRad)
//                 + fieldRelativeSpeeds.vyMetersPerSecond * Math.cos(angleRad);
//     }

//     /* ================= VISION ================= */

//     private void updateVision() {

//         double turretAngleDeg = turret.getTurretAngleRelative();
//         double turretAngleRad = Math.toRadians(turretAngleDeg);

//         double camForward = TURRET_PIVOT_FORWARD + (CAMERA_FROM_PIVOT_AXIAL * Math.cos(turretAngleRad));
//         double camSide    = TURRET_PIVOT_SIDE    - (CAMERA_FROM_PIVOT_AXIAL * Math.sin(turretAngleRad));

//         LimelightLib.setCameraPose_RobotSpace(
//                 LIMELIGHT,
//                 camForward,
//                 camSide,
//                 CAMERA_HEIGHT,
//                 0.0,
//                 CAMERA_PITCH,
//                 turretAngleDeg
//         );

//         LimelightLib.SetRobotOrientation(
//                 LIMELIGHT,
//                 getGyroRotation().getDegrees(),
//                 getTurnRate(),
//                 0, 0, 0, 0
//         );

//         PoseEstimate vision = LimelightLib.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);

//         if (!LimelightLib.validPoseEstimate(vision)) return;

//         if (Math.abs(getTurnRate()) > 720) return;

//         // MegaTag2 outputs poses in WPILib blue-origin coordinates natively,
//         // so vision.pose.getTranslation() is always correct for both alliances.
//         // We override rotation with the raw gyro — never trust MegaTag2 for heading.
//         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
//         poseEstimator.addVisionMeasurement(
//                 new Pose2d(vision.pose.getTranslation(), getGyroRotation()),
//                 vision.timestampSeconds
//         );

//         field.getObject("VisionPose").setPose(vision.pose);
//     }

//     /* ================= FIELD CALCULATIONS ================= */

//     /**
//      * Returns the alliance-aware field center target.
//      * Red alliance uses the mirrored X; blue alliance uses the base X.
//      */
//     private Translation2d getFieldCenter() {
//         double x = FIELD_CENTER_X;
//         boolean isRed = DriverStation.getAlliance().isPresent()
//                 && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
//         if (isRed) {
//             x = FIELD_LENGTH - x;
//         }
//         return new Translation2d(x, FIELD_WIDTH / 2.0);
//     }

//     private void updateFieldCalculations() {

//         Pose2d pose = poseEstimator.getEstimatedPosition();

//         // For Field2d display, flip the pose for red alliance so the robot
//         // appears on the correct side of the field in Shuffleboard/Elastic.
//         // This is purely cosmetic — the underlying pose estimator stays in
//         // blue-origin coordinates for PathPlanner compatibility.
//         boolean isRed = DriverStation.getAlliance().isPresent()
//                 && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
//         Pose2d displayPose = isRed
//                 ? new Pose2d(
//                     new Translation2d(FIELD_LENGTH - pose.getX(), pose.getY()),
//                     pose.getRotation().plus(Rotation2d.fromDegrees(180)))
//                 : pose;
//         field.setRobotPose(displayPose);

//         // All game-logic calculations (angleToCenter, distanceToCenter) use the
//         // raw blue-origin pose so they remain consistent with the pose estimator.
//         double robotHeading = getGyroRotation().getRadians();
//         double turretWorldX = pose.getX()
//                 + TURRET_PIVOT_FORWARD * Math.cos(robotHeading)
//                 - TURRET_PIVOT_SIDE   * Math.sin(robotHeading);
//         double turretWorldY = pose.getY()
//                 + TURRET_PIVOT_FORWARD * Math.sin(robotHeading)
//                 + TURRET_PIVOT_SIDE   * Math.cos(robotHeading);

//         Translation2d turretPivot = new Translation2d(turretWorldX, turretWorldY);
//         Translation2d fieldCenter = getFieldCenter();

//         distanceToCenter = turretPivot.getDistance(fieldCenter);

//         double dx = fieldCenter.getX() - turretPivot.getX();
//         double dy = fieldCenter.getY() - turretPivot.getY();

//         double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
//         angleToCenter = MathUtil.inputModulus(getGyroRotation().getDegrees() - fieldAngle, -180, 180);

//         // Draw markers using display pose for Field2d consistency
//         Translation2d displayTurretPivot = isRed
//                 ? new Translation2d(FIELD_LENGTH - turretWorldX, turretWorldY)
//                 : turretPivot;
//         Translation2d displayFieldCenter = isRed
//                 ? new Translation2d(FIELD_LENGTH - fieldCenter.getX(), fieldCenter.getY())
//                 : fieldCenter;

//         field.getObject("FieldCenter").setPose(new Pose2d(displayFieldCenter, new Rotation2d()));
//         field.getObject("TurretPivot").setPose(new Pose2d(displayTurretPivot, displayPose.getRotation()));
//         field.getObject("ToCenter").setPoses(List.of(
//                 new Pose2d(displayTurretPivot, new Rotation2d()),
//                 new Pose2d(displayFieldCenter, new Rotation2d())));
//     }

//     /* ================= HUB STATUS ================= */

//     private void updateHubStatus() {
//         var alliance = DriverStation.getAlliance();

//         if (alliance.isEmpty()) {
//             hubActive = false;
//             hubStatus = "Unknown";
//             return;
//         }

//         if (DriverStation.isAutonomousEnabled()) {
//             hubActive = true;
//             hubStatus = "Active (Auto)";
//             currentShift = 0;
//             shiftTimeRemaining = DriverStation.getMatchTime();
//             return;
//         }

//         if (!DriverStation.isTeleopEnabled()) {
//             hubActive = false;
//             hubStatus = "Inactive";
//             currentShift = 0;
//             shiftTimeRemaining = 0;
//             return;
//         }

//         double t = DriverStation.getMatchTime();
//         String gameData = DriverStation.getGameSpecificMessage();

//         if (gameData.isEmpty()) {
//             hubActive = true;
//             hubStatus = "Active";
//             return;
//         }

//         boolean redInactiveFirst = gameData.charAt(0) == 'R';
//         boolean isRed = alliance.get() == DriverStation.Alliance.Red;

//         boolean shift1Active = isRed ? !redInactiveFirst : redInactiveFirst;

//         if (t > 130) {
//             hubActive = true;
//             currentShift = 0;
//             shiftTimeRemaining = t - 130;
//         } else if (t > 105) {
//             hubActive = shift1Active;
//             currentShift = 1;
//             shiftTimeRemaining = t - 105;
//         } else if (t > 80) {
//             hubActive = !shift1Active;
//             currentShift = 2;
//             shiftTimeRemaining = t - 80;
//         } else if (t > 55) {
//             hubActive = shift1Active;
//             currentShift = 3;
//             shiftTimeRemaining = t - 55;
//         } else if (t > 30) {
//             hubActive = !shift1Active;
//             currentShift = 4;
//             shiftTimeRemaining = t - 30;
//         } else {
//             hubActive = true;
//             currentShift = 5;
//             shiftTimeRemaining = t;
//         }

//         hubStatus = hubActive ? "Active" : "Inactive";
//     }

//     /* ================= DASHBOARD ================= */

//     private void updateDashboard() {
//         SmartDashboard.putNumber("Robot Heading",        getHeading());
//         SmartDashboard.putNumber("Angle To Center",      angleToCenter);
//         SmartDashboard.putNumber("Distance To Center",   distanceToCenter);
//         SmartDashboard.putNumber("Lateral Velocity",     getLateralVelocityToTarget());
//         SmartDashboard.putNumber("RR",                   rearRight.getAngle());
//         SmartDashboard.putNumber("RL",                   rearLeft.getAngle());
//         SmartDashboard.putNumber("RRoffset",             rearRight.getAngleFull());
//         SmartDashboard.putNumber("RLoffset",             rearLeft.getAngleFull());
//         SmartDashboard.putBoolean("Hub Active",          hubActive);
//         SmartDashboard.putString("Hub Status",           hubStatus);
//         SmartDashboard.putNumber("Hub Shift",            currentShift);
//         SmartDashboard.putNumber("Shift Time Remaining", shiftTimeRemaining);
//     }

//     /* ================= POSE METHODS ================= */

//     public Pose2d getPose() {
//         return poseEstimator.getEstimatedPosition();
//     }

//     public void resetOdometry(Pose2d pose) {
//         poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
//     }

//     public double getAngleToCenter()    { return angleToCenter; }
//     public double getDistanceToCenter() { return distanceToCenter; }

//     /* ================= DRIVE ================= */

//     public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

//         double x = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
//         double y = yLimiter.calculate(ySpeed)  * DriveConstants.kMaxSpeedMetersPerSecond;
//         double r = rotLimiter.calculate(rot)   * DriveConstants.kMaxAngularSpeed;

//         ChassisSpeeds speeds = fieldRelative
//                 ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, getGyroRotation())
//                 : new ChassisSpeeds(x, y, r);

//         setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
//     }

//     public void driveRobotRelative(ChassisSpeeds speeds) {
//         setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
//     }

//     public void driveLocked(double xSpeed, double ySpeed, double targetHeading, boolean fieldRelative) {
//         double correction = headingCorrector.calculate(getHeading(), targetHeading);
//         drive(xSpeed, ySpeed, correction, fieldRelative);
//     }

//     /* ================= MODULE CONTROL ================= */

//     public void setModuleStates(SwerveModuleState[] states) {
//         SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
//         frontLeft.setDesiredState(states[0]);
//         frontRight.setDesiredState(states[1]);
//         rearLeft.setDesiredState(states[2]);
//         rearRight.setDesiredState(states[3]);
//     }

//     public ChassisSpeeds getRobotRelativeSpeeds() {
//         return DriveConstants.kDriveKinematics.toChassisSpeeds(
//                 frontLeft.getState(),
//                 frontRight.getState(),
//                 rearLeft.getState(),
//                 rearRight.getState()
//         );
//     }

//     /* ================= GYRO ================= */

//     public void zeroHeading() { gyro.reset(); }

//     public double getHeading() {
//         return MathUtil.inputModulus(getGyroRotation().getDegrees(), -180, 180);
//     }

//     public double getTurnRate() { return -gyro.getRate(); }

//     /**
//      * Single gyro method — no alliance offset.
//      * Used everywhere: pose estimator, vision, field calculations, and driving.
//      * PathPlanner's alliance-flip lambda handles red-alliance path mirroring.
//      */
//     public Rotation2d getGyroRotation() {
//         return Rotation2d.fromDegrees(-gyro.getAngle());
//     }

//     /* ================= UTIL ================= */

//     public SwerveModulePosition[] getModulePositions() {
//         return new SwerveModulePosition[]{
//                 frontLeft.getPosition(),
//                 frontRight.getPosition(),
//                 rearLeft.getPosition(),
//                 rearRight.getPosition()
//         };
//     }

//     public void resetEncoders() {
//         frontLeft.resetEncoders();
//         frontRight.resetEncoders();
//         rearLeft.resetEncoders();
//         rearRight.resetEncoders();
//     }
// }