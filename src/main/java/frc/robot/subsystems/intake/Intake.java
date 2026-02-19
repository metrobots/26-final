package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {

    // Create the intake motors
    private SparkMax intakePivot = new SparkMax(Constants.kIntakePivotCanId, MotorType.kBrushless);
    private SparkMax intakeDrive = new SparkMax(Constants.kIntakeDriveCanId, MotorType.kBrushless);
    private SparkMax indexer = new SparkMax(Constants.kIndexerCanId, MotorType.kBrushless);

    double kP = 0.000001;
    double kI = 0;
    double kD = 0;

    PIDController intakePID = new PIDController(kP, kI, kD);

    // Spindexer States
    public enum IndexerState {
        ACTIVE,
        ACTIVE_INTAKING,
        INACTIVE,
        MANUAL_SPIN,
        ERROR
    }

    IndexerState currentState = IndexerState.ACTIVE;
    
    public void periodic () {
        switch (currentState) {
            case ACTIVE:
                spinIndexer(0);
            case ACTIVE_INTAKING:
                spinIndexer(0.5);
            case INACTIVE:
                spinIndexer(0);
            case MANUAL_SPIN:
                spinIndexer(1);
            case ERROR:
                indexer.stopMotor();
        }
    }

    public void setIndexerState (IndexerState requestedState) {
        currentState = requestedState;
        return;
    }

    public void indexer (boolean intaking) {
        if (currentState == IndexerState.ACTIVE && intaking) {
            setIndexerState (IndexerState.ACTIVE_INTAKING);
        } else if (currentState == IndexerState.ACTIVE_INTAKING && !intaking) {
            setIndexerState (IndexerState.ACTIVE);
        }
        return;
    }

    public double getEncoder() {
        return intakePivot.getEncoder().getPosition();
    }

    // Define methods to move the motors forwards and backwards
    public void spinIndexer(double speed) {
        indexer.set(speed);
    }

    public void toAngle(double setpoint) {
        double speed = intakePID.calculate(intakePivot.getEncoder().getPosition(), setpoint);
        intakePivot.set(speed);
    }

    public void manualPivot(double speed) {
        intakePivot.set(speed);
    }

    public void driveIntake(double speed) {
        intakeDrive.set(speed);
    }

    public void pivotIntake(double setpoint) {
        intakePivot.set(intakePID.calculate(intakePivot.getEncoder().getPosition(), setpoint));
        intakePID.close();
    }

    public void homeIntake() {
        intakePivot.getEncoder().setPosition(0);
    }
    
}
