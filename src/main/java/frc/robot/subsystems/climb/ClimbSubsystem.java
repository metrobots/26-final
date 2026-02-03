package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ClimbSubsystem extends SubsystemBase {
    // Define motors and such for climb
    private final SparkMax climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
    private final AbsoluteEncoder encoder = climbMotor.getAbsoluteEncoder();
    private final ProfiledPIDController pidController = new ProfiledPIDController(
            1.0, // p
            0.0, // i
            0.0, // d
            new TrapezoidProfile.Constraints(10, 20) // TODO: Figure out what these values should be.
    );

    public ClimbSubsystem() {
        pidController.reset(0);
        configureMotor();
    }

    public void configureMotor() {
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        // TODO: Calculate convertion factor.
        encoderConfig.positionConversionFactor(1);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.apply(encoderConfig);
        climbMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    /**
     * Drives the motor used for climb. A positive value means climb is going up and
     * a negative value indicates climb is going down.
     * TODO: ^^^^ Check this ^^^^
     * 
     * @param speed Speed of the motor (between -1.0 and 1.0).
     */
    public void driveMotor(double speed) {
        climbMotor.set(pidController.calculate(getPosition(), speed));
    }

    /**
     * TODO: Figure out units.
     * 
     * @return The encoder value in UNITS.
     */
    public double getPosition() {
        return encoder.getPosition();
    }
}
