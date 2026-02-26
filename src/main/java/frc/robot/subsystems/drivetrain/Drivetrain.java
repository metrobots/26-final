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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  // Modules
  private final Module m_frontLeft = new Module(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftEncoder,
      DriveConstants.kFrontLeftEncoderOffset);

  private final Module m_frontRight = new Module(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightEncoder,
      DriveConstants.kFrontRightEncoderOffset);

  private final Module m_rearLeft = new Module(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftEncoder,
      DriveConstants.kRearLeftEncoderOffset);

  private final Module m_rearRight = new Module(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightEncoder,
      DriveConstants.kRearRightEncoderOffset);

  // Gyro (CCW positive assumed)
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Pose Estimator (with tuned std devs)
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          getGyroRotation(),
          getModulePositions(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Math.toRadians(1)),
          VecBuilder.fill(0.7, 0.7, Math.toRadians(10)));

  private final Field2d field = new Field2d();

  private final PIDController headingCorrector = new PIDController(0, 0, 0);

  private final SlewRateLimiter xSpeedLimiter =
      new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter ySpeedLimiter =
      new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter rotLimiter =
      new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  public Drivetrain() {

    SmartDashboard.putData("Field", field);

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              new PIDConstants(5, 0.0, 0.0),
              new PIDConstants(5, 0.0, 0.0)),
          config,
          () -> DriverStation.getAlliance()
                  .map(a -> a == DriverStation.Alliance.Red)
                  .orElse(false),
          this);

    } catch (Exception e) {
      DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getGyroRotation(),
        getModulePositions());

    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Heading", getHeading());
  }

  // ===========================
  // Pose / Odometry
  // ===========================

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        getModulePositions(),
        pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }

  // ===========================
  // Driving
  // ===========================

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double xSpeedLimited = xSpeedLimiter.calculate(xSpeed);
    double ySpeedLimited = ySpeedLimiter.calculate(ySpeed);
    double rotLimited = rotLimiter.calculate(rot);

    double xDelivered = xSpeedLimited * DriveConstants.kMaxSpeedMetersPerSecond;
    double yDelivered = ySpeedLimited * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rotLimited * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xDelivered,
            yDelivered,
            rotDelivered,
            getGyroRotation())
        : new ChassisSpeeds(xDelivered, yDelivered, rotDelivered);

    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(states);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(states);
  }

  public void driveLocked(double xSpeed, double ySpeed, double targetHeading, boolean fieldRelative) {
    double correction = headingCorrector.calculate(getHeading(), targetHeading);
    correction = MathUtil.clamp(correction, -1, 1);
    drive(xSpeed, ySpeed, correction, fieldRelative);
  }

  private void setModuleStates(SwerveModuleState[] states) {
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  // ===========================
  // Helpers
  // ===========================

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()); // remove negative unless needed
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return MathUtil.inputModulus(m_gyro.getAngle(), -180.0, 180.0);
  }

  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }
}