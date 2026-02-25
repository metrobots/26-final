package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;

public class TurretLeading extends Command {

  private final Turret turret;
  private final Drivetrain drivetrain;

  private static final String LIMELIGHT_NAME = "limelight-front";

  // TUNE THIS VALUE PLEASE PLEASE PLEASE PLEASE
  private static final double kLead = 0;

  /*
    STEPS FOR TUNING:
    Tune turret PID while stationary.
    Drive sideways at 1 m/s.
    Shoot.
    Increase kLead until drift disappears.
    If shots are consistently off in one direction flip the sign of kLead.
   */


  private static final PIDController turretPID =
      new PIDController(0.0001, 0.0, 0.0);

  public TurretLeading(Turret turret, Drivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;
    addRequirements(turret);
  }

  @Override
  public void execute() {

    if (!LimelightLib.getTV(LIMELIGHT_NAME)) {
      turret.manualTurret(0);
      return;
    }

    double tx = LimelightLib.getTX(LIMELIGHT_NAME);

    // Robot-relative speeds
    ChassisSpeeds speeds = drivetrain.getRobotRelativeSpeeds();

    // Sideways velocity (meters/sec)
    double lateralVelocity = speeds.vyMetersPerSecond;

    // Simple velocity lead (no distance)
    double leadOffset = kLead * lateralVelocity;

    // Apply correction
    double correctedTx = tx + leadOffset;

    double output = turretPID.calculate(correctedTx, 0);

    turret.manualTurret(output);
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