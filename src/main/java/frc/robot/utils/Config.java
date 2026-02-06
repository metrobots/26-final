package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.utils.Constants.ModuleConstants;

public final class Config {

  public static final class Module {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();


    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI /
        ModuleConstants.kDrivingMotorReduction;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
      drivingConfig.encoder
        .positionConversionFactor(drivingFactor) // meters
        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0, 0)
        .velocityFF(drivingVelocityFeedForward)
        .outputRange(-1, 1);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .inverted(true);
      turningConfig.encoder
        .positionConversionFactor(2 * Math.PI / ModuleConstants.kTurningMotorReduction) // rotations
        .velocityConversionFactor(2 * Math.PI / (ModuleConstants.kTurningMotorReduction * 60.0)); // rotations per second
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1, 0, 0.0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2*Math.PI);
    }
  }
}