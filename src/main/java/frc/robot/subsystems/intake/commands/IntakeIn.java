package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeIn extends Command {

  private final Intake intake;
  private final double speed;

  /**
   * @param intake Intake subsystem
   * @param speed  Motor output (-1.0 to 1.0), keep this small (ex: ±0.15)
   */
  public IntakeIn(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.driveIntake(speed);
    intake.indexer(true);
  }

  @Override
  public void end(boolean interrupted) {
    intake.driveIntake(0.0);
    intake.indexer(false);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until button released
  }
}
