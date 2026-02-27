package frc.robot.subsystems.climb;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    // TODO: Move to Constants.java
    /* Climb motor constants. */
    // The climb motor has a gear ratio of 64:1 (4:1 -> 4:1 -> 4:1).
    private static final double gearRatio = 64.0;
    // The climb motor has a drum radius of 2 inches.
    private static final double circumeference = 4 * Math.PI;
    // Convert from rotations to meters.
    private static final double positionConversionFactor = circumeference / gearRatio;
    // Convert from rpm to in/s.
    private static final double velocityConversionFactor = positionConversionFactor / 60;

    /* Climb mechanism constants. */
    public static final double maxExtensionInInches = 8;

    public Climb() {
        climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
        encoder = climbMotor.getEncoder();
        closedLoopController = climbMotor.getClosedLoopController();
        configure();
        // TODO: Check this.
        //  When the robot is turned on, the climb mechanism is completely collapsed.
        encoder.setPosition(0);
    }

    private void configure() {
        motorConfig
                // Inverted so + goes up, - goes down
                .inverted(true);
        motorConfig.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);
        motorConfig.closedLoop
                .p(0.5)
                .i(0)
                .d(0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1.0, 1.0);
        climbMotor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * @return The extension of the climb mechanism in inches.
     */
    public double getExtension() {
        return encoder.getPosition();
    }

    /**
     * Set the desired extension (not height) of the climb mechanism.
     * 
     * @param target The target extension in inches.
     */
    public void setDesiredExtension(double target) {
        closedLoopController.setSetpoint(target, ControlType.kPosition);
    }

    // public void getEncoder() {
    //     // return encoder;
    // }

    // public double getMaxHeight() {
    //     return maxHeight;
    // }

    // public double getMinHeight() {
    //     return minHeight;
    // }

    // public double getMaxExtension() {
    //     return maxExtension;
    // }

    // // Positional movement
    // public void positionalMove(double setpoint, double speed) {

    //     // climbMotor.set(climbPID.calculate(speed, setpoint));

    //     return;
    // }

    /**
     * Manually move the climb.
     * 
     * @param speed -1.0 to 1.0
     *              positive = up
     *              negative = down
     */
    public void move(double speed) {
        climbMotor.set(speed);
    }

    /**
     * Stops the climb motor.
     */
    public void stop() {
        climbMotor.stopMotor();
    }
}