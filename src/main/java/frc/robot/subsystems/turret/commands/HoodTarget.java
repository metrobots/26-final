// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodTarget extends Command {
  /** Creates a new ManualTurret. */

  Turret turret;
  double input;
  private final PIDController control = new PIDController(0.18, 0, 0);

  public HoodTarget(Turret turret, double input) {
      this.turret = turret;
      this.input = input;
      addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  double angle = turret.hoodEncoder.getPosition();

  SmartDashboard.putNumber("MATEO", angle);

  double output = control.calculate(angle, input);

  turret.hoodSpark.set(output);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.manualTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
