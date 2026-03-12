package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public final RelativeEncoder flywheelEncoder;
    public final RelativeEncoder hoodEncoder;
    public final AbsoluteEncoder turretAbsoluteEncoder;
    public final RelativeEncoder turretRelativeEncoder;
    public final RelativeEncoder feedEncoder;

    // ---------------- CONSTANTS ----------------
    private static final double GEAR_RATIO = 0.03659432;  // Motor:Turret
    private static final double ABS_ENCODER_ZERO_OFFSET = 0; //0.9970865249633789

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
            .smartCurrentLimit(80)
            .voltageCompensation(12);
            

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
        feedConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(80);
        feedConfig.voltageCompensation(12.0);
        feedConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0 / 60.0);
        feedSpark.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- TURRET CONFIG ----------------
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        turretSpark.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- ENCODERS ----------------
        flywheelEncoder = flywheelSpark1.getEncoder();
        hoodEncoder = hoodSpark.getEncoder();
        turretAbsoluteEncoder = turretSpark.getAbsoluteEncoder();
        turretRelativeEncoder = turretSpark.getEncoder(); // relative motor encoder
        feedEncoder = feedSpark.getEncoder();

        // ---------------- CALIBRATE RELATIVE ENCODER ----------------
        calibrateTurretEncoder();
    }

    /**
     * Syncs the relative encoder with the absolute encoder at startup.
     */
    public void calibrateTurretEncoder() {
      Timer.delay(0.5);
      double absFraction = turretAbsoluteEncoder.getPosition();
      double correctedFraction = absFraction - ABS_ENCODER_ZERO_OFFSET;
      double motorRotations = -correctedFraction * 8;
      turretRelativeEncoder.setPosition(motorRotations);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle (deg)", getTurretAngle());
        SmartDashboard.putNumber("Turret Angle Relative (deg)", getTurretAngleRelative());
        SmartDashboard.putNumber("Turret Encoder Raw", turretAbsoluteEncoder.getPosition());
    }

    // =========================
    // ===== FLYWHEEL ==========
    // =========================
    public void setFlywheelVoltage(double volts) {
        flywheelSpark1.setVoltage(volts);
    }

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

    public double getHood() {
        return hoodEncoder.getPosition();
    }

    public void manualTurret(double input) {
        double output = MathUtil.clamp(input, -1.0, 1.0);
        turretSpark.set(output);
    }

    public void spinFeed(double speed) {
        feedSpark.setVoltage(speed);
    }

    public double getFeedVelocity() {
        return feedEncoder.getVelocity();
    }

    // =========================
    // ===== TURRET ANGLES =====
    // =========================

    /**
     * Returns turret angle in degrees, 0-360.
     * Forward = 0, right = increasing, left = wraps from 360 downward.
     */
    public double getTurretAngle() {
        double motorRotations = turretRelativeEncoder.getPosition();

        // invert sign so CCW is positive
        double turretRotations = -motorRotations * GEAR_RATIO;

        double turretDegrees = turretRotations * 360.0;

        // normalize to 0-360
        turretDegrees = ((turretDegrees % 360.0) + 360.0) % 360.0;

        return turretDegrees;
    }

    /**
     * Returns signed turret angle in degrees, -180 to 180.
     * 0 = forward, positive = CCW (left), negative = CW (right).
     */
    public double getTurretAngleRelative() {
        double angle = getTurretAngle();
        return ((angle + 180.0) % 360.0) - 180.0;
    }
}