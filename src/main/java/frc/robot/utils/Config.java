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
// Conversion factors
double drivingFactor =
        ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;

double turningFactor =
        2.0 * Math.PI / ModuleConstants.kTurningMotorReduction;

double nominalVoltage = 12.0;
double drivingVelocityFeedForward =
        nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

/* ===================== DRIVING ===================== */

drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

drivingConfig.encoder
        .positionConversionFactor(drivingFactor)              // meters
        .velocityConversionFactor(drivingFactor / 60.0);      // meters/sec

drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .feedForward.kV(drivingVelocityFeedForward);

/* ===================== TURNING ===================== */

turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .inverted(true);

turningConfig.encoder
        .positionConversionFactor(turningFactor)              // radians
        .velocityConversionFactor(turningFactor / 60.0);      // rad/sec

turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 2.0 * Math.PI);
}
}
}
