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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    /* Climb mechanism constants. */

    public Climb() {
        climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
        encoder = climbMotor.getEncoder();
        closedLoopController = climbMotor.getClosedLoopController();
        configure();
        //  When the robot is turned on, the climb mechanism is completely collapsed.
        encoder.setPosition(0);
    }

    private void configure() {
        motorConfig
                // Inverted so + goes up, - goes down
                .inverted(true);
        motorConfig.encoder
                .positionConversionFactor(ClimbConstants.positionConversionFactor)
                .velocityConversionFactor(ClimbConstants.velocityConversionFactor);
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

    /**
     * Stops the climb motor.
     */
    public void stop() {
        climbMotor.stopMotor();
    }
}
