// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.LimelightLib;
import frc.robot.subsystems.drivetrain.Drivetrain;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTurret extends Command {
  private final Turret turret;
private final Drivetrain drivetrain;
  private static final String LIMELIGHT_NAME = "limelight-front";
  private static final PIDController turretPID = new PIDController(0.01  , 0, 0); //est start at 0.02

  /** Creates a new TestTurret. */
  public RotateTurret(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double targetAngle = drivetrain.getAngleToCenter();  // hub direction
      double currentAngle = turret.getTurretAngle();

      SmartDashboard.putNumber("Turret Target", targetAngle);
      SmartDashboard.putNumber("Turret Angle", currentAngle);

      double output = -turretPID.calculate(currentAngle, targetAngle);

      SmartDashboard.putNumber("Turret Output", output);

      boolean atLeftLimit = currentAngle > 110;
      boolean atRightLimit = currentAngle < -110;

      boolean tryingToMoveLeft = output > 0;
      boolean tryingToMoveRight = output < 0;

      if ((atLeftLimit && tryingToMoveLeft) ||
          (atRightLimit && tryingToMoveRight)) {

          turret.manualTurret(0);  // block outward motion
      } 
      else {
          turret.manualTurret(output);
      }
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
