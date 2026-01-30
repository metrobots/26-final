package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Turret extends SubsystemBase {
    // Motors
    public final SparkFlex flywheelSpark1; 
    public final SparkFlex flywheelSpark2;
    public final SparkMax hoodSpark; 
    public final SparkMax feedSpark;
    public final SparkMax turretSpark;

    // Encoders
    private final RelativeEncoder flywheelEncoder;
    private final AbsoluteEncoder hoodEncoder;
    private final RelativeEncoder turretEncoder;
    private final DigitalInput turretLimitSwitch;

    // Constants
    private final double maxHoodAngle = 90;
    private final double minHoodAngle = 0;
    private final double maxTurretAngle = 40; // max rotation each side
    private final double flywheelNominalVoltage = 12.0;

    // Configurations
    private final SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig turretMotorConfig = new SparkMaxConfig();

    public Turret() {
        // Motor initialization
        flywheelSpark1 = new SparkFlex(Constants.kFlywheel1CanId, MotorType.kBrushless);
        flywheelSpark2 = new SparkFlex(Constants.kFlywheel2CanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(Constants.kHoodCanId, MotorType.kBrushless);
        feedSpark = new SparkMax(Constants.kFeedCanId, MotorType.kBrushless);
        turretSpark = new SparkMax(Constants.kTurretCanId, MotorType.kBrushless);

        turretLimitSwitch = new DigitalInput(Constants.kTurretLimitSwitchPort);

        // --- HOOD CONFIG ---
        double hoodFactor = 2 * Math.PI; // radians per rotation

        hoodMotorConfig.idleMode(IdleMode.kBrake)
                       .smartCurrentLimit(20);

        hoodMotorConfig.absoluteEncoder
                       .inverted(false)
                       .positionConversionFactor(hoodFactor)
                       .velocityConversionFactor(hoodFactor / 60.0)
                       .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

        hoodMotorConfig.closedLoop
                       .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                       .pid(1, 0, 0) // tune PID
                       .positionWrappingEnabled(false);

        hoodSpark.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- FLYWHEEL CONFIG ---
        double flywheelFreeSpeedRPM = 5000.0; // replace with your motor free speed
        double flywheelFreeSpeedRPS = flywheelFreeSpeedRPM / 60.0;
        double kV = flywheelNominalVoltage / flywheelFreeSpeedRPS;

        flywheelMotorConfig.idleMode(IdleMode.kCoast)
                           .smartCurrentLimit(40);

        flywheelMotorConfig.encoder
                          .positionConversionFactor(1.0)
                          .velocityConversionFactor(1.0 / 60.0);

        flywheelMotorConfig.closedLoop
                          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .pid(0.0005, 0, 0)
                          .feedForward.kV(kV);

        flywheelSpark1.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelSpark2.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- TURRET CONFIG ---
        double turretPositionFactor = 1.0; // rotations per rotation (adjust with gear ratio)
        double turretVelocityFactor = 1.0 / 60.0; // RPM to RPS

        turretMotorConfig.idleMode(IdleMode.kBrake)
                         .smartCurrentLimit(30);

        turretMotorConfig.encoder
                        .positionConversionFactor(turretPositionFactor)
                        .velocityConversionFactor(turretVelocityFactor);

        turretMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.5, 0, 0); // tune PID

        turretSpark.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- ENCODERS ---
        flywheelEncoder = flywheelSpark1.getEncoder();
        hoodEncoder = hoodSpark.getAbsoluteEncoder();
        turretEncoder = turretSpark.getEncoder();
    }

    // --- HOOD CONTROL ---
    public void manualHood(double input) {
        // Scale and clamp voltage to respect min/max hood angle
        double currentAngle = hoodEncoder.getPosition();
        if ((input > 0 && currentAngle >= maxHoodAngle) || (input < 0 && currentAngle <= minHoodAngle)) {
            hoodSpark.setVoltage(0);
        } else {
            hoodSpark.setVoltage(MathUtil.clamp(input, -1.0, 1.0) * flywheelNominalVoltage);
        }
    }

    public void setHoodAngle(double setpoint) {
        double clampedSetpoint = MathUtil.clamp(setpoint, minHoodAngle, maxHoodAngle);
        hoodSpark.getClosedLoopController().setSetpoint(clampedSetpoint, ControlType.kPosition);
    }

    // --- TURRET CONTROL ---
    public void manualTurret(double input) {
        double currentAngle = turretEncoder.getPosition();
        double voltage = MathUtil.clamp(input, -1.0, 1.0) * flywheelNominalVoltage;

        // Prevent movement past physical limits
        if ((voltage > 0 && currentAngle >= maxTurretAngle) || 
            (voltage < 0 && (currentAngle <= -maxTurretAngle || turretAtLimit()))) {
            voltage = 0;
        }

        turretSpark.setVoltage(voltage);
    }

    public void setTurretAngle(double setpoint) {
        double clampedSetpoint = MathUtil.clamp(setpoint, -maxTurretAngle, maxTurretAngle);
        turretSpark.getClosedLoopController().setSetpoint(clampedSetpoint, ControlType.kPosition);
    }

    public boolean turretAtLimit() {
        return !turretLimitSwitch.get(); // pressed = false
    }

    // --- FLYWHEEL CONTROL ---
    public void manualFlywheels(double speed) {
        flywheelSpark1.setVoltage(speed * flywheelNominalVoltage);
        flywheelSpark2.setVoltage(speed * flywheelNominalVoltage);
    }

    public void setFlywheelRPM(double rpm) {
        double targetRPS = rpm / 60.0;
        flywheelSpark1.getClosedLoopController().setSetpoint(targetRPS, ControlType.kVelocity);
        flywheelSpark2.getClosedLoopController().setSetpoint(targetRPS, ControlType.kVelocity);
    }

    // --- FEED CONTROL ---
    public void spinFeed(double speed) {
        feedSpark.setVoltage(speed * flywheelNominalVoltage);
    }

    public double getTurretAngle() {
        return turretEncoder.getPosition();
    }

    public double getHoodAngle() {
        return hoodEncoder.getPosition();
    }

    public boolean  homeTurret() {
    final double homingSpeed = -0.2 * flywheelNominalVoltage;

    if (!turretAtLimit()) {
        turretSpark.setVoltage(homingSpeed);
        return false; // not done yet
    } else {
        turretSpark.setVoltage(0);
        turretEncoder.setPosition(0);
        return true; // homing finished
    }
    }
}
