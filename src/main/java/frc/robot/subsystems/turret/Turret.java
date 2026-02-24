package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
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
    private final AbsoluteEncoder turretEncoder;
    private final RelativeEncoder feedEncoder;

    // Constants
    private final double maxTurretAngle = 40;

    // Configs
    private final SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig turretMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig feedConfig = new SparkMaxConfig();

    // ---------------- FLYWHEEL CONSTANTS (used by commands) ----------------
    public static final double kFlywheelTargetRPM = 2000.0;
    public static final double kFlywheelAtSpeedThresholdRPM = 50.0;
    public static final double kFlywheelKp = 0.001;
    public static final double kFlywheelKv = 12.0 / (6784.0 / 60.0); // 12V / free speed RPS

    @SuppressWarnings("removal")
    public Turret() {
        // Motor initialization
        flywheelSpark1 = new SparkFlex(Constants.kFlywheel1CanId, MotorType.kBrushless);
        flywheelSpark2 = new SparkFlex(Constants.kFlywheel2CanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(Constants.kHoodCanId, MotorType.kBrushless);
        feedSpark = new SparkMax(Constants.kFeedCanId, MotorType.kBrushless);
        turretSpark = new SparkMax(Constants.kTurretCanId, MotorType.kBrushless);

        // ---------------- HOOD CONFIG ----------------
        hoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        hoodMotorConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        hoodMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1, 0, 0)
                .positionWrappingEnabled(false);
        hoodSpark.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ---------------- FEED CONFIG ----------------
        feedConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        feedSpark.configure(
            feedConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        SparkMaxConfig flywheelMasterConfig = new SparkMaxConfig();
        SparkMaxConfig flywheelFollowerConfig = new SparkMaxConfig();

        flywheelMasterConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40);

        flywheelMasterConfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);

        flywheelMasterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kFlywheelKp, 0, 0)
                .feedForward.kV(kFlywheelKv);

        // Follower config
        flywheelFollowerConfig
                .apply(flywheelMasterConfig)
                .follow(flywheelSpark1, true);  // true = inverted follower

        // Apply configs
        flywheelSpark1.configure(
                flywheelMasterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        flywheelSpark2.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // ---------------- TURRET CONFIG ----------------
        turretMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        turretMotorConfig.absoluteEncoder
                .positionConversionFactor(1.0);
        turretMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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


    // ---------------- TURRET ----------------
    public void manualTurret(double input) {
        double angle = turretEncoder.getPosition();
        double voltage = MathUtil.clamp(input, -1.0, 1.0);
        if (( angle >= maxTurretAngle) || (angle <= -maxTurretAngle)) {
            voltage = 0;
        }
        turretSpark.setVoltage(voltage);
    }

    // ---------------- FLYWHEEL ----------------
    public void manualFlywheels(double speed) {
        flywheelSpark1.set(speed);
    }

    public double getFlywheelRPM() {
        return flywheelEncoder.getVelocity(); // RPS → RPM
    }

    // ---------------- FEED ----------------
    public void spinFeed(double speed) {
        feedSpark.set(speed);
    }

    public double getFeedRPM() {
        return feedEncoder.getVelocity(); // RPS → RPM
    }

    // ---------------- ACCESSORS ----------------
    public double getTurretAngle() { return turretEncoder.getPosition(); }
    public double getHoodAngle() { return hoodEncoder.getPosition(); }
}
