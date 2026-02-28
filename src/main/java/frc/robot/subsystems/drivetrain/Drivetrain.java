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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  // Create the modules
  // Front Left
  private final Module m_frontLeft = new Module(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftEncoder,
      DriveConstants.kFrontLeftEncoderOffset);

  // Front Right
  private final Module m_frontRight = new Module(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightEncoder,
      DriveConstants.kFrontRightEncoderOffset);

  // Rear Left
  private final Module m_rearLeft = new Module(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftEncoder,
      DriveConstants.kRearLeftEncoderOffset);

  // Rear Right
  private final Module m_rearRight = new Module(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightEncoder,
      DriveConstants.kRearRightEncoderOffset);

  // Create gyro (NavX)
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // PID controller for heading
  private PIDController headingCorrector = new PIDController(0, 0, 0);

  private static Field2d field = Constants.AutoConstants.field2d;

  // Ban people from going directly forward to backward immediately
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  // Odometry for tracking pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // Pose estimator for tracking pose again
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(-m_gyro.getAngle()),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d()  // Initial pose
  );

  public Drivetrain() {

    // Pathplanner things
    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetOdometry,
        this::getRobotRelativeSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          new PIDConstants(5, 0.0, 0.0), // Translation PID constants (LEAVE AT 5)
          new PIDConstants(5, 0.0, 0.0) // Rotation PID constants (LEAVE AT 5)
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {

    // Update the odometry with wheel encoders and gyro
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update the pose estimator with wheel encoders and gyro
    m_poseEstimator.update(
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

      // Display the estimated positions for odometry and pose estimator on the field
      field.getObject("Odometry").setPose(m_odometry.getPoseMeters());
      field.getObject("Pose Estimator").setPose(m_poseEstimator.getEstimatedPosition());


      SmartDashboard.putNumber("FR", m_frontRight.getAngleFull());
      SmartDashboard.putNumber("FL", m_frontLeft.getAngleFull());
      SmartDashboard.putNumber("RL", m_rearLeft.getAngleFull());
      SmartDashboard.putNumber("RR", m_rearRight.getAngleFull());
      SmartDashboard.putNumber("angle", m_gyro.getAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();
    // Invert coordinates to match the transformation in resetOdometry
    return new Pose2d(
        -estimatedPose.getX(),
        -estimatedPose.getY(),
        estimatedPose.getRotation()
    );
  }

  /**
   * Returns the current robot-relative chassis speeds
   * 
   * @return ChassisSpeeds object containing the robot's velocity components
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the current states of each swerve module
    SwerveModuleState frontLeftState = m_frontLeft.getState();
    SwerveModuleState frontRightState = m_frontRight.getState();
    SwerveModuleState rearLeftState = m_rearLeft.getState();
    SwerveModuleState rearRightState = m_rearRight.getState();

    // Convert the swerve module states to chassis speeds
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  /**
   * Drives the robot with the specified robot-relative chassis speeds.
   * 
   * @param speeds The desired robot-relative chassis speeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
      // 
      ChassisSpeeds invertedSpeeds = new ChassisSpeeds(
          -speeds.vxMetersPerSecond,
          -speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond
      );
      
      // Convert the inverted speeds to swerve module states
      SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(invertedSpeeds);

      // Desaturate the wheel speeds
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

      // Set the desired state for each swerve module
      setModuleStates(moduleStates);
  }

  public void resetWheels() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
      // Invert the coordinates to match the transformation in getPose()
      Pose2d invertedPose = new Pose2d(
          -pose.getX(),
          -pose.getY(),
          pose.getRotation()
      );
      
      // Reset the pose estimator
      m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        invertedPose
    );

    // Reset the odometry
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        invertedPose
    );
  }

  /**
   * Method to drive the robot while holding an angle.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param targetHeading The heading to hold the drivetrain to
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveLocked(double xSpeed, double ySpeed, double targetHeading, boolean fieldRelative) {
      // Calculate rotation correction to maintain heading
      double currentHeading = getHeading();
      double rotationCorrection = headingCorrector.calculate(currentHeading, targetHeading);
     
      // Clamp the rotation correction between -1 and 1
      rotationCorrection = Math.max(-1, Math.min(1, rotationCorrection));
     
      // Use the normal drive method with the correction
      drive(xSpeed, ySpeed, rotationCorrection, fieldRelative);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Apply rate limiting
    double xSpeedLimited = xSpeedLimiter.calculate(xSpeed);
    double ySpeedLimited = ySpeedLimiter.calculate(ySpeed);
    double rotLimited = rotLimiter.calculate(rot);

    // Convert to drivetrain speeds
    double xSpeedDelivered = xSpeedLimited * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedLimited * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rotLimited * DriveConstants.kMaxAngularSpeed;

    // Convert to chassis speeds
    var chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // Convert to swerve module states
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply to modules
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    // Get the angle from the gyro
    double angle = -m_gyro.getAngle();
    
    // Wrap to -180 to 180 range
    angle = MathUtil.inputModulus(angle, -180.0, 180.0);
    
    return angle;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public Rotation2d getGyroRotation() {
      return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      };
  }
}