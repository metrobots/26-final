package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RotateTurret extends Command {

  private final Turret turret;
  private final Drivetrain drivetrain;

  private final PIDController turretPID =
      new PIDController(0.005, 0.0, 0.0);

  private static final double MAX_TURRET = 40;
  private static final double MIN_TURRET = -40;

  public RotateTurret(Turret turret, Drivetrain drivetrain) {

    this.turret = turret;
    this.drivetrain = drivetrain;

    turretPID.setTolerance(2.0);

    addRequirements(turret);
  }

  @Override
  public void execute() {

    double targetAngle = drivetrain.getAngleToCenter();
    double currentAngle = turret.getTurretAngleRelative();


    double output = turretPID.calculate(currentAngle, targetAngle);

    output = -MathUtil.clamp(output, -0.6, 0.6);

    SmartDashboard.putNumber("Turret Target", targetAngle);
    SmartDashboard.putNumber("Turret Angle", currentAngle);
    SmartDashboard.putNumber("Turret Output", output);

    boolean atLeftLimit = currentAngle >= MAX_TURRET;
    boolean atRightLimit = currentAngle <= MIN_TURRET;

    if ((atLeftLimit && output > 0) || (atRightLimit && output < 0)) {
      turret.manualTurret(0);
    } else {
      turret.manualTurret(output);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.manualTurret(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}