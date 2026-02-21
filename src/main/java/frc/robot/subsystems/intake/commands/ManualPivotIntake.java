package frc.robot.subsystems.intake.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class ManualPivotIntake extends Command {

  private final Intake intake;
  // PIDController intakePID = new PIDController(0.01, 0, 0);

  /**
   * @param intake Intake subsystem
   */
  public ManualPivotIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    // double setpoint = 40;
    // SmartDashboard.putNumber("intakePoint", intake.getEncoder());

    // double speed = intakePID.calculate(setpoint, intake.getEncoder());
    intake.intakePivot.set(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakePivot.set(0.0);
    // intakePID.close();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until button released
  }
}
