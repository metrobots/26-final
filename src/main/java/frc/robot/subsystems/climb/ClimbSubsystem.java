package frc.robot.subsystems.climb;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

/**
 * The climb subsystem consists of a single motor which lifts the climb
 * mechanism up and down. The mechanism will hook onto the tower bar and raise
 * the robot. The robot can only do L1 climb.
 * 
 * For the controller bindings, I think holding the climb button to ascend and releasing
 * climb button to pull the robot up should work. I could also make it a more autonomous process
 * only requiring a button press (see commented out getAutoClimbCommand()).
 */
public class ClimbSubsystem extends SubsystemBase {
    /* Motors, encoders, and configs. */
    private final SparkMax climbMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    // TODO: Move to Config.java
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    // 2 inch drum , 4π inch circumference.
    private final int radius = 2;
    private final double circumference = 2 * Math.PI * radius;
    // Gear reduction of 64:1 (4:1 -> 4:1 -> 4:1)
    private final double gearRatio = 64.0;
    private final double positionConversionFactor = circumference / gearRatio;

    // TODO: Move to Constants.java
    /** The distance from the bottom of climb to the top in inches with the arm fully extended. */
    public static double CLIMB_MAX_HEIGHT = 27.0;
    /** The distance from the bottom of climb to the top in inches with the arm fully collapsed. */
    public static double CLIMB_MIN_HEIGHT = 20.0;
    /** The maximum extension of the climb mechanism. */
    public static double CLIMB_MAX_EXTENSION = CLIMB_MAX_HEIGHT - CLIMB_MIN_HEIGHT;

    public ClimbSubsystem() {
        climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
        encoder = climbMotor.getEncoder();
        closedLoopController = climbMotor.getClosedLoopController();
        configure();
        encoder.setPosition(0);
    }

    /**
     * Configures the motor and encoder.
     */
    public void configure() {
        motorConfig
                // Inverted so + is up and - is down.
                .inverted(true);
        motorConfig.encoder
                // Rotations to inches.
                .positionConversionFactor(positionConversionFactor)
                // Rotations per minute to inches / second
                .velocityConversionFactor(positionConversionFactor / 60);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.5)
                .i(0)
                .d(0)
                .outputRange(-1.0, 1.0);
        climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * @return The total extension relative to bottommost position in inches.
     */
    public double getExtensionInInches() {
        return encoder.getPosition();
    }

    /**
     * Sets the desired height (in inches) for the top of the climb mechanism. 
     */
    public void setDesiredExtension(double desiredHeight) {
        closedLoopController.setSetpoint(desiredHeight, ControlType.kPosition);
    }

    /**
     * Sets the motor speed to zero. Idle behavior should break, causing the climb mechanism to stay
     * in place.
     */
    public void stopMotor() {
        climbMotor.stopMotor();
    }
}
