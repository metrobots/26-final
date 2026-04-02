// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.turret.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.turret.Turret;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ManualHood extends Command {
//   /** Creates a new ManualTurret. */
//   Turret m_turret;
//   double m_input;

//   private final PIDController hoodPID = new PIDController(0.001, 0, 0);

//   public ManualHood(Turret turret, double input) {
//     this.m_turret = turret;
//     this.m_input = input;
//     addRequirements(turret);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     // SmartDashboard.putBoolean("mateo", true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     SmartDashboard.putBoolean("mateo", true);
//     // double angle = m_turret.getHood();
//     // SmartDashboard.putNumber("mateo", angle);
//     // double output = hoodPID.calculate(angle, m_input);

//     // m_turret.hoodSpark.set(output);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // m_turret.manualHood(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
