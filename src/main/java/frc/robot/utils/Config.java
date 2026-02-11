package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.utils.Constants.ModuleConstants;

public final class Config {

  public static final class Module {

    private Module() {}

    /* =======================
     * DRIVING CONFIG FACTORY
     * ======================= */
    public static SparkMaxConfig createDrivingConfig() {
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI /
          ModuleConstants.kDrivingMotorReduction;

      double drivingVelocityFeedForward =
          1.0 / ModuleConstants.kDriveWheelFreeSpeedRps;

      SparkMaxConfig config = new SparkMaxConfig();

      config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(drivingFactor)           // meters
        .velocityConversionFactor(drivingFactor / 60.0);   // meters/sec

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0.0, 0.0)
        .velocityFF(drivingVelocityFeedForward)
        .outputRange(-1.0, 1.0);

      return config;
    }

    /* =======================
     * TURNING CONFIG FACTORY
     * ======================= */
    public static SparkMaxConfig createTurningConfig() {
      SparkMaxConfig config = new SparkMaxConfig();

      config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .inverted(true);

      config.encoder
        .positionConversionFactor(
            2 * Math.PI / ModuleConstants.kTurningMotorReduction)
        .velocityConversionFactor(
            2 * Math.PI / (ModuleConstants.kTurningMotorReduction * 60.0));

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 2 * Math.PI);

      return config;
    }
  }
}
