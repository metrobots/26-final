package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.utils.Constants;

public class Turret extends SubsystemBase {

    // ---------------- MOTORS ----------------
    public final SparkFlex flywheelSpark1;
    public final SparkFlex flywheelSpark2;
    public final SparkMax hoodSpark;
    public final SparkMax feedSpark;
    public final SparkMax turretSpark;

    private final RelativeEncoder flywheelEncoder;
    private final AbsoluteEncoder hoodEncoder;
    private final AbsoluteEncoder turretEncoder;
    private final RelativeEncoder feedEncoder;

    private final double maxTurretAngle = 40;

    // Store applied voltage for SysId logging
    private double lastFlywheelVoltage = 0.0;

    // ---------------- CONSTRUCTOR ----------------
    @SuppressWarnings("removal")
    public Turret() {

        flywheelSpark1 = new SparkFlex(Constants.kFlywheel1CanId, MotorType.kBrushless);
        flywheelSpark2 = new SparkFlex(Constants.kFlywheel2CanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(Constants.kHoodCanId, MotorType.kBrushless);
        feedSpark = new SparkMax(Constants.kFeedCanId, MotorType.kBrushless);
        turretSpark = new SparkMax(Constants.kTurretCanId, MotorType.kBrushless);

        // ---------------- FLYWHEEL CONFIG ----------------
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        masterConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        // Velocity in RPS
        masterConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0 / 60.0);

        followerConfig
            .apply(masterConfig)
            .follow(flywheelSpark1, true);

        flywheelSpark1.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelSpark2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- HOOD CONFIG ----------------
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        hoodSpark.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- FEED CONFIG ----------------
        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        feedSpark.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- TURRET CONFIG ----------------
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        turretSpark.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- ENCODERS ----------------
        flywheelEncoder = flywheelSpark1.getEncoder();
        hoodEncoder = hoodSpark.getAbsoluteEncoder();
        turretEncoder = turretSpark.getAbsoluteEncoder();
        feedEncoder = feedSpark.getEncoder();
    }

    // =========================
    // ===== FLYWHEEL ==========
    // =========================

    public void setFlywheelVoltage(double volts) {
        lastFlywheelVoltage = volts;
        flywheelSpark1.set(volts);
    }

    // Returns RPS
    public double getFlywheelVelocity() {
        return flywheelEncoder.getVelocity();
    }

    public double getFlywheelPosition() {
        return flywheelEncoder.getPosition();
    }

    // =========================
    // ===== OTHER SYSTEMS =====
    // =========================

    public void manualHood(double input) {
        hoodSpark.set(input);
    }

    public void manualTurret(double input) {
        double angle = turretEncoder.getPosition();
        double output = MathUtil.clamp(input, -1.0, 1.0);

        if (angle >= maxTurretAngle || angle <= -maxTurretAngle) {
            output = 0;
        }

        turretSpark.set(output);
    }

    public void spinFeed(double speed) {
        feedSpark.set(speed);
    }

    public double getFeedVelocity() {
        return feedEncoder.getVelocity();
    }

    public double getTurretAngle() {
        return turretEncoder.getPosition();
    }

    public double getHoodAngle() {
        return hoodEncoder.getPosition();
    }
}