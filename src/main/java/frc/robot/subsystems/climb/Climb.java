package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climb extends SubsystemBase {

    private final SparkMax climbMotor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public Climb() {
        climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
        configure();
    }

    private void configure() {
        motorConfig
            // Inverted so + goes up, - goes down
            .inverted(true);

        climbMotor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

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