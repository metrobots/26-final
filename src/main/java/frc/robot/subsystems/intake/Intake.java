package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.subsystems.dashboard.Dashboard;

public class Intake extends SubsystemBase {
    // Dahboard so we can tell hub states
    Dashboard dashboard;
    Alliance alliance; 

    // Create the intake motors
    public SparkMax intakePivot = new SparkMax(Constants.kIntakePivotCanId, MotorType.kBrushless);
    public SparkMax intakeDrive = new SparkMax(Constants.kIntakeDriveCanId, MotorType.kBrushless);
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
        INACTIVE_INTAKING,
        MANUAL_SPIN,
        ERROR
    }

    public IndexerState currentState = IndexerState.ACTIVE;

    public Intake (Dashboard dashboard) {
        this.dashboard = dashboard;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = Alliance.Blue;
        } else {
            alliance = Alliance.Red;
        }
    }
    
    public void periodic () {
        switch (currentState) {
            case ACTIVE:
                spinIndexer(0);
            case ACTIVE_INTAKING:
                spinIndexer(0.5);
            case INACTIVE:
                spinIndexer(0);
            case INACTIVE_INTAKING:
                spinIndexer(0.1);
                spinIndexer(-0.1);
            case MANUAL_SPIN:
                spinIndexer(1);
            case ERROR:
                indexer.stopMotor();
        }
        if ((alliance == Alliance.Red && dashboard.isRedHubActive()) || (alliance == Alliance.Blue && dashboard.isBlueHubActive())) {
            currentState = IndexerState.ACTIVE;
        } else {
            currentState = IndexerState.INACTIVE;
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
        } else if (currentState == IndexerState.INACTIVE && intaking) {
            setIndexerState (IndexerState.INACTIVE_INTAKING);
        } else if (currentState == IndexerState.INACTIVE_INTAKING && !intaking) {
            setIndexerState(IndexerState.INACTIVE);
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
