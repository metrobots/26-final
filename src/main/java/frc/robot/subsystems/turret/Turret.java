package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit.SoftLimitDirection;
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
import frc.robot.utils.LimelightLib;

public class Turret extends SubsystemBase {
    // Motors
    public final SparkFlex flywheelSpark1; 
    public final SparkMax hoodSpark; 
    public final SparkMax feedSpark;
    public final SparkMax turretSpark;

    // Encoders
    private final RelativeEncoder flywheelEncoder;
    private final AbsoluteEncoder hoodEncoder;
    private final AbsoluteEncoder turretEncoder;
    private final RelativeEncoder feedEncoder;

    // Constants
    private final double maxHoodAngle = 90;
    private final double minHoodAngle = 0;
    private final double maxTurretAngle = 40;
    private final double flywheelNominalVoltage = 12.0;

    // Target flywheel RPM
    private double targetFlywheelRPM = 0;

    // Configs
    private final SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig invertedFlywheelMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig turretMotorConfig = new SparkMaxConfig();

    // Turret PID constants for Limelight
    private static final double kTurretTxKp = 1.0;
    private static final double kTxDeadband = 0.5;

    public Turret() {
        // Motor initialization
        flywheelSpark1 = new SparkFlex(Constants.kFlywheel1CanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(Constants.kHoodCanId, MotorType.kBrushless);
        feedSpark = new SparkMax(Constants.kFeedCanId, MotorType.kBrushless);
        turretSpark = new SparkMax(Constants.kTurretCanId, MotorType.kBrushless);

        // ---------------- HOOD CONFIG ----------------
        double hoodFactor = 2 * Math.PI; // radians per rotation
        hoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        hoodMotorConfig.absoluteEncoder
                .inverted(false)
                .positionConversionFactor(hoodFactor)
                .velocityConversionFactor(hoodFactor / 60.0)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        hoodMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .positionWrappingEnabled(false);
        hoodSpark.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- FLYWHEEL CONFIG ----------------
        double flywheelFreeSpeedRPS = 6784.0 / 60.0; // RPM to RPS
        double kV = flywheelNominalVoltage / flywheelFreeSpeedRPS;

        flywheelMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        flywheelMotorConfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);
        flywheelMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0005, 0, 0)
                .feedForward.kV(kV);

        invertedFlywheelMotorConfig.apply(flywheelMotorConfig);
        invertedFlywheelMotorConfig.inverted(true);

        flywheelSpark1.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- TURRET CONFIG ----------------
        turretMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        turretMotorConfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);
        turretMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.5, 0, 0);
        turretSpark.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // ---------------- ENCODERS ----------------
        flywheelEncoder = flywheelSpark1.getEncoder();
        hoodEncoder = hoodSpark.getAbsoluteEncoder();
        turretEncoder = turretSpark.getAbsoluteEncoder();
        feedEncoder = feedSpark.getEncoder();
    }

    // ---------------- HOOD ----------------
    public void manualHood(double input) {
        hoodSpark.set(input);
    }

     public double getFeedRPM() {
        return feedEncoder.getVelocity() * 60; // RPS → RPM
    }

    public void setHoodAngle(double setpoint) {
        double clamped = MathUtil.clamp(setpoint, minHoodAngle, maxHoodAngle);
        hoodSpark.getClosedLoopController().setSetpoint(clamped, ControlType.kPosition);
    }

    // ---------------- TURRET ----------------
    public void manualTurret(double input) {
        double angle = turretEncoder.getPosition();
        double voltage = MathUtil.clamp(input, -1.0, 1.0);
        if (( angle >= maxTurretAngle) || (angle <= -maxTurretAngle)) {
            voltage = 0;
        }
        turretSpark.setVoltage(voltage);
    }

    public void setTurretAngle(double setpoint) {
        double clamped = MathUtil.clamp(setpoint, -maxTurretAngle, maxTurretAngle);
        turretSpark.getClosedLoopController().setSetpoint(clamped, ControlType.kPosition);
    }

    public void aimTurretWithTx() {
        double tx = LimelightLib.getTX("limelight");
        if (!LimelightLib.getTV("limelight") || Math.abs(tx) < kTxDeadband) return;
        double target = MathUtil.clamp(turretEncoder.getPosition() + tx * kTurretTxKp, -maxTurretAngle, maxTurretAngle);
        turretSpark.getClosedLoopController().setSetpoint(target, ControlType.kPosition);
    }

    // ---------------- FLYWHEEL ----------------
    public void manualFlywheels(double speed) {
        flywheelSpark1.set(speed);
    }

    public void setFlywheelRPM(double rpm) {
        double targetRPS = rpm / 60.0;
        flywheelSpark1.getClosedLoopController().setSetpoint(targetRPS, ControlType.kVelocity);
    }

    public void setTargetFlywheelRPM(double rpm) {
        targetFlywheelRPM = rpm;
    }

    public double getTargetFlywheelRPM() {
        return targetFlywheelRPM;
    }

    public double getFlywheelRPM() {
        return flywheelEncoder.getVelocity() * 60; // RPS → RPM
    }

    public boolean isFlywheelAtTarget() {
        return Math.abs(getFlywheelRPM() - targetFlywheelRPM) < 50; // RPM tolerance
    }

    // ---------------- FEED ----------------
    public void spinFeed(double speed) {
        feedSpark.set(speed);
    } //test changes

    // ---------------- ACCESSORS ----------------
    public double getTurretAngle() { return turretEncoder.getPosition(); }
    public double getHoodAngle() { return hoodEncoder.getPosition(); }
}
