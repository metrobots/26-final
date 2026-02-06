package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ClimbSubsystem extends SubsystemBase {
    // Define motors and controllers for climb.
    private final SparkMax climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
    private final AbsoluteEncoder encoder = climbMotor.getAbsoluteEncoder();
    private final ProfiledPIDController pidController = new ProfiledPIDController(
            1.0, // p
            0.0, // i
            0.0, // d
            new TrapezoidProfile.Constraints(10, 20) // TODO: Figure out what these values should be.
    );

    // The climb motor has a gear-reduction ratio of 64:1 and a 2" radius drum.
    private final int radius = 2;
    private final double circumference = 2 * Math.PI * radius;
    private final double gearRatio = 64 / 1;

    public ClimbSubsystem() {
        pidController.reset(0);
        configureMotor();
    }

    /**
     * Configures the position conversion factor for climb motor.
     */
    public void configureMotor() {
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.positionConversionFactor(circumference / gearRatio);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.apply(encoderConfig);
        climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Drives the motor used for climb. A positive value means climb is going up and
     * a negative value indicates climb is going down.
     * TODO: ^^^^ Check this ^^^^
     * 
     * @param speed Speed of the motor (between -1.0 and 1.0).
     */
    public void driveMotor(double speed) {
        climbMotor.set(pidController.calculate(getPositionInInches(), speed));
    }

    /**
     * @return The height of climb value in inches.
     */
    public double getPositionInInches() {
        return encoder.getPosition();
    }
}
