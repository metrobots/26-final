package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
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
    public final SparkMax  hoodSpark;
    public final SparkMax  feedSpark;
    public final SparkMax  turretSpark;

    // ---------------- ENCODERS ----------------
    public final RelativeEncoder   flywheelEncoder;
    public final RelativeEncoder   hoodEncoder;
    public final AbsoluteEncoder   turretAbsoluteEncoder;
    public final RelativeEncoder   turretRelativeEncoder;
    public final RelativeEncoder   feedEncoder;

    // ---------------- ONBOARD FLYWHEEL PID ----------------
    public final SparkClosedLoopController flywheelController;

    // Flywheel PID gains — tuned on the SPARK Flex at ~1000 Hz.
    // Start with these and adjust kP until you get fast recovery without oscillation.
    // kFF replaces the software SimpleMotorFeedforward: kFF = 1 / (free-speed RPS).
    // For a NEO Vortex free speed ~94 RPS: kFF ≈ 1/94 ≈ 0.0106. Tune from there.
    private static final double FLYWHEEL_kP  = 0.05;   // TUNE — same starting point as before
    private static final double FLYWHEEL_kI  = 0.0;
    private static final double FLYWHEEL_kD  = 0.0;
    private static final double FLYWHEEL_kFF = 0.011; // TUNE — replaces SimpleMotorFeedforward

    // ---------------- TURRET CONSTANTS ----------------
    private static final double GEAR_RATIO          = 0.035;  // Motor:Turret
    private static final double ABS_ENCODER_ZERO_OFFSET = 0.0;

    // ---------------- CONSTRUCTOR ----------------
    @SuppressWarnings("removal")
    public Turret() {

        flywheelSpark1 = new SparkFlex(Constants.kFlywheel1CanId, MotorType.kBrushless);
        flywheelSpark2 = new SparkFlex(Constants.kFlywheel2CanId, MotorType.kBrushless);
        hoodSpark      = new SparkMax(Constants.kHoodCanId,    MotorType.kBrushless);
        feedSpark      = new SparkMax(Constants.kFeedCanId,    MotorType.kBrushless);
        turretSpark    = new SparkMax(Constants.kTurretCanId,  MotorType.kBrushless);

        // ---------------- FLYWHEEL CONFIG ----------------
        SparkFlexConfig masterConfig   = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();

        masterConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80)
            .voltageCompensation(12);

        masterConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0 / 60.0); // RPM → RPS

        // Configure onboard closed-loop on the master controller.
        // ControlType.kVelocity runs at ~1000 Hz on the SPARK Flex.
        masterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(FLYWHEEL_kP)
            .i(FLYWHEEL_kI)
            .d(FLYWHEEL_kD)
            .velocityFF(FLYWHEEL_kFF)
            .outputRange(-1.0, 1.0);

        followerConfig
            .apply(masterConfig)
            .follow(flywheelSpark1, true); // inverted follower

        flywheelSpark1.configure(masterConfig,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelSpark2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Grab the onboard PID controller from the master
        flywheelController = flywheelSpark1.getClosedLoopController();

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
        flywheelEncoder       = flywheelSpark1.getEncoder();
        hoodEncoder           = hoodSpark.getEncoder();
        turretAbsoluteEncoder = turretSpark.getAbsoluteEncoder();
        turretRelativeEncoder = turretSpark.getEncoder();
        feedEncoder           = feedSpark.getEncoder();

        // ---------------- CALIBRATE RELATIVE ENCODER ----------------
        calibrateTurretEncoder();
    }

    /**
     * Syncs the relative encoder with the absolute encoder at startup.
     */
    public void calibrateTurretEncoder() {
        double absFraction = turretAbsoluteEncoder.getPosition();
        double correctedFraction = absFraction - ABS_ENCODER_ZERO_OFFSET;
        correctedFraction = ((correctedFraction + 0.5) % 1.0 + 1.0) % 1.0 - 0.5;
        double motorRotations = -correctedFraction * 8.0;
        turretRelativeEncoder.setPosition(motorRotations);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle (deg)",         getTurretAngle());
        SmartDashboard.putNumber("Turret Angle Relative (deg)", getTurretAngleRelative());
        SmartDashboard.putNumber("Turret Encoder Raw",          turretAbsoluteEncoder.getPosition());
    }

    // =========================
    // ===== FLYWHEEL ==========
    // =========================

    /**
     * Commands the onboard PID controller to hold a target velocity in RPS.
     * The SPARK Flex runs this loop at ~1000 Hz — no software PID or FF needed
     * in the command; kFF on the controller handles steady-state.
     */
    public void setFlywheelVelocity(double targetRPS) {
        flywheelController.setReference(targetRPS, ControlType.kVelocity);
    }

    /**
     * Drives the flywheel at a raw voltage, bypassing the onboard PID entirely.
     * Used by TuneFeedforwards to isolate FF response without PID correction.
     */
    public void setFlywheelVoltage(double volts) {
        flywheelSpark1.setVoltage(volts);
    }

    /** Stop the flywheel immediately by commanding 0 V. */
    public void stopFlywheel() {
        flywheelSpark1.setVoltage(0);
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
        turretSpark.set(MathUtil.clamp(input, -1.0, 1.0));
    }

    public void spinFeed(double voltage) {
        feedSpark.setVoltage(voltage);
    }

    public double getFeedVelocity() {
        return feedEncoder.getVelocity();
    }

    // =========================
    // ===== TURRET ANGLES =====
    // =========================

    public double getTurretAngle() {
        double encoderRotations = turretRelativeEncoder.getPosition();
        double turretRotations  = -encoderRotations * GEAR_RATIO;
        double turretDegrees    = turretRotations * 360.0;
        return ((turretDegrees % 360.0) + 360.0) % 360.0;
    }

    public double getTurretAngleRelative() {
        double angle = getTurretAngle();
        return ((angle + 180.0) % 360.0) - 180.0;
    }
}