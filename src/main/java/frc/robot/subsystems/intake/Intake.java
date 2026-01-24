package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {

    // Create the intake motors
    private SparkMax intakePivot = new SparkMax(Constants.kIntakePivotCanId, MotorType.kBrushless);
    private SparkMax intakeDrive = new SparkMax(Constants.kIntakeDriveCanId, MotorType.kBrushless);
    private SparkMax indexer = new SparkMax(Constants.kIndexerCanId, MotorType.kBrushless);

    private AbsoluteEncoder pivotEncoder = intakePivot.getAbsoluteEncoder();

    double kP = 0.000001;
    double kI = 0;
    double kD = 0;

    PIDController intakePID = new PIDController(kP, kI, kD);

    // Define methods to move the motors forwards and backwards
    public void spinIndexer(double speed) {
        indexer.set(speed);
    }

    public void driveIntake(double speed) {
        intakeDrive.set(speed);
    }

    public void pivotIntake(double setpoint) {
        intakePivot.set(intakePID.calculate(pivotEncoder.getPosition(), setpoint));
        intakePID.close();
    }
    
}
