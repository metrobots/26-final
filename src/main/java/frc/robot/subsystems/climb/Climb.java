package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climb extends SubsystemBase {

    private static final PIDController climbPID = new PIDController(0, 0, 0);
    private final RelativeEncoder encoder;

    private final SparkMax climbMotor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final double maxHeight;
    private final double minHeight;
    private final double maxExtension;

    public Climb() {

        climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
        encoder = climbMotor.getEncoder();
        configure();

        // placeholder values; tunning is required.
        this.maxHeight = 20.0;
        this.minHeight = 10.0;
        this.maxExtension = 21.0;
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

    public void getEncoder() {
        return encoder;
    }

    public double getMaxHeight() {
        return maxHeight;
    }

    public double getMinHeight() {
        return minHeight;
    }

    public double getMaxExtension() {
        return maxExtension;
    }

    // Positional movement
    public void positionalMove(double setpoint, double speed) {

        climbMotor.set(climbPID.calculate(speed, setpoint));

        return;
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