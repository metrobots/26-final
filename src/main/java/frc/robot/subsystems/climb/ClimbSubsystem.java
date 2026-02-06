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

/**
 * The climb subsystem consists of a single motor which lifts the climb
 * mechanism up and down. The mechanism will hook onto the tower bar and raise
 * the robot.
 */
public class ClimbSubsystem extends SubsystemBase {
    // Define motors and controllers for climb.
    private final SparkMax climbMotor = new SparkMax(Constants.kClimbCanId, MotorType.kBrushless);
    private final AbsoluteEncoder encoder = climbMotor.getAbsoluteEncoder();
    // TODO: Move configs to here.
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

    // Going to move these to Constants.java once merged.
    public static final double MOTOR_SPEED = 0.5;

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
        motorConfig.inverted(true);
        climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Drives the climb motor towards a desired height.
     * 
     * @param desiredHeight The desired height (in inches) of the climb mechanism.
     */
    public void moveTowardsHeight(double desiredHeight) {
        double targetSpeed = pidController.calculate(getPositionInInches(), desiredHeight);
        climbMotor.set(targetSpeed);
    }

    /**
     * @return The height the bottom of the climb mechanism from its starting point.
     */
    public double getPositionInInches() {
        return encoder.getPosition();
    }

    /**
     * Sets the speed of the climb motor to 0.
     */
    public void stopMotor() {
        climbMotor.stopMotor();
    }
}
