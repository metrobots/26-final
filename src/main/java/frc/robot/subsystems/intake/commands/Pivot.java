// package frc.robot.subsystems.intake.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.intake.Intake;

// public class Pivot extends Command {

//   private final Intake intake;
//   private final double setpoint; // Placeholder

//   /**
//    * @param intake Intake subsystem
//    * @param speed  Motor output (-1.0 to 1.0), keep this small (ex: ±0.15)
//    */
//   public Pivot (Intake intake, double setpoint) {
//     this.intake = intake;
//     this.setpoint = setpoint;
//     addRequirements(intake);
//   }

//   @Override
//   public void execute() {
//     intake.pivotIntake(setpoint);
//     intake.indexer(true);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     intake.intakePivot.stopMotor();
//     intake.indexer(false);
//   }

//   @Override
//   public boolean isFinished() {
//     return false; // runs until button released
//   }
// }

