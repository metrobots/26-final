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


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTurret extends Command {
  private final Turret turret;
  private static final String LIMELIGHT_NAME = "limelight-front";
  private static final PIDController turretPID = new PIDController(0.014  , 0, 0.002); //est start at 0.02

  /** Creates a new TestTurret. */
  public RotateTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
  }

  // Called every time the scheduler runs while the command is scheduled.
 @Override
public void execute() {

    // If no target, stop turret
    if (!LimelightLib.getTV(LIMELIGHT_NAME)) {
        turret.manualTurret(0);
        return;
    }

    if (LimelightLib.getFiducialID(LIMELIGHT_NAME) == 26) {

    double tx = LimelightLib.getTX(LIMELIGHT_NAME);
    double angle = turret.getTurretAngle();

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("Turret Angle", angle);

    // Calculate PID output (negative if your motor direction requires it)
    double output = -turretPID.calculate(tx, 0);

    boolean atLeftLimit = angle > 110;
    boolean atRightLimit = angle < -110;

    boolean tryingToMoveLeft = output > 0;
    boolean tryingToMoveRight = output < 0;

    if ((atLeftLimit && tryingToMoveLeft) ||
        (atRightLimit && tryingToMoveRight)) {

        turret.manualTurret(0);  // Block outward motion
    } 
    else {
        turret.manualTurret(output);  // Allow safe motion
    }
    }

    // ----- Soft Limit Protection -----
    // Prevent moving further OUT of bounds,
    // but allow moving back inward.
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
