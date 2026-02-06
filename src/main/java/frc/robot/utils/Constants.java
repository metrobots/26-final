package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {

  public static CommandXboxController primary = new CommandXboxController(OIConstants.kDriverControllerPort);

  // INTAKE - 
  public static final int kIntakePivotCanId = 21;
  public static final int kIntakeDriveCanId = 22;

  // SHOOTER - 
  public static final int kTurretCanId = 30;
  public static final int kFeedCanId = 31;
  public static final int kHoodCanId = 32;
  public static final int kFlywheel1CanId = 33;
  public static final int kFlywheel2CanId = 34;

  // INDEXER -
  public static final int kIndexerCanId = 35;

  // CLIMB -
  public static final int kClimbCanId = 50;

  public static final int kTurretLimitSwitchPort = 0;

  public static final class DriveConstants {
    // Be careful, these are the max allowed speeds, not the max capable
    public static final double kMaxSpeedMetersPerSecond = 5.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians/second

    // Chassis Config

    // Distance between the center of right and left modules 
    public static final double kTrackWidth = Units.inchesToMeters(23.75); // This is correct for 2025 drivetrain
    // Distance between the center of the front and back modules
    public static final double kWheelBase = Units.inchesToMeters(23.75);

    // IF IT DOES NOT WORK REVERT THE KINEMATICS
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), 
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   
        new Translation2d(kWheelBase / 2, kTrackWidth / 2)     
    );

    // Drivetrain CAN IDs




    // by module 
    public static final int kFrontLeftDrivingCanId = 40;
    public static final int kFrontLeftTurningCanId = 41;

    public static final int kRearLeftDrivingCanId = 42;
    public static final int kRearLeftTurningCanId = 43;


    public static final int kFrontRightDrivingCanId = 44;
    public static final int kFrontRightTurningCanId = 45;

    public static final int kRearRightDrivingCanId = 46;
    public static final int kRearRightTurningCanId = 47;

    // Encoders
    public static final int kFrontLeftEncoder = 0; 
    public static final int kFrontRightEncoder = 3;
    public static final int kRearLeftEncoder = 1; 
    public static final int kRearRightEncoder = 2;
    // Encoder offsets
    public static final double kFrontLeftEncoderOffset = 0;
    public static final double kFrontRightEncoderOffset = 0;
    public static final double kRearLeftEncoderOffset = 0;
    public static final double kRearRightEncoderOffset = 0;




  }

  public static final class ModuleConstants {
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i gear ratio
    public static final double kDrivingMotorReduction = 6.12; // L3 ratio
    public static final double kTurningMotorReduction = 21.4285714286; // MK4i ratio

    // Motor attributes
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
      kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Controller port
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {

    public static final Field2d field2d = new Field2d();

    static {
        // Initialize once at startup
        SmartDashboard.putData("Field", field2d);
    }

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constrain the motion profiled controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

      public static final String limelightName = "limelight-front";
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class PIDConstants {
    public static final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 
    public static final PIDController xPID = new PIDController(0.5, 0.0, 0.00); 
  }
}