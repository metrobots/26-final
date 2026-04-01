// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldZero extends Command {
  private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);
  private final PIDController hoodPID   = new PIDController(0.18, 0.0, 0.0);
  Turret turret;
  /** Creates a new HoldZero. */
  public HoldZero(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double currentAngle     = turret.getTurretAngleRelative();

      double turretOutput = MathUtil.clamp(
          turretPID.calculate(currentAngle, 0),
          -0.6, 0.6
      );

      turret.manualTurret(-turretOutput);

        double hoodOutput = hoodPID.calculate(turret.hoodEncoder.getPosition(), 0);
        turret.manualHood(hoodOutput);
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
