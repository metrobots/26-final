// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnTurret extends Command {
  double speed;
  private final Turret turret;
  /** Creates a new TurnTurret. */
  public TurnTurret(double input, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.speed = input;
    addRequirements(turret);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.manualTurret(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
