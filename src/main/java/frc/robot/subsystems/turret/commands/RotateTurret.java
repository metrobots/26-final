package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;
import java.util.Set;

public class RotateTurret extends Command {
  private final Turret turret;
  private final Drivetrain drivetrain;
  private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);

  private static final String LIMELIGHT_NAME = "limelight-front";
  private static final double MAX_TURRET = 40.0;
  private static final double MIN_TURRET = -40.0;

  // Edit these to match the AprilTag IDs you want to track
  private static final Set<Integer> VALID_TARGET_IDS = Set.of(10, 26);

  public RotateTurret(Turret turret, Drivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;
    turretPID.setTolerance(2.0);
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turretPID.reset();
  }

  private boolean hasValidLimelightTarget() {
    return LimelightLib.getTV(LIMELIGHT_NAME)
        && VALID_TARGET_IDS.contains((int) LimelightLib.getFiducialID(LIMELIGHT_NAME));
  }

  @Override
  public void execute() {
    boolean usingLimelight = hasValidLimelightTarget();
    double targetAngle;

    if (usingLimelight) {
      // tx = horizontal angle offset from crosshair to target; 0 means turret is centered
      targetAngle = MathUtil.clamp(LimelightLib.getTX(LIMELIGHT_NAME), MIN_TURRET, MAX_TURRET);
    } else {
      targetAngle = MathUtil.clamp(drivetrain.getAngleToCenter(), MIN_TURRET, MAX_TURRET);
    }

    double currentAngle = turret.getTurretAngleRelative();
    double output = turretPID.calculate(currentAngle, targetAngle);
    output = MathUtil.clamp(output, -0.6, 0.6);

    SmartDashboard.putNumber("Turret Target", targetAngle);
    SmartDashboard.putNumber("Turret Current Angle", currentAngle);
    SmartDashboard.putNumber("Turret Output", output);
    SmartDashboard.putBoolean("Turret At Target", turretPID.atSetpoint());
    SmartDashboard.putBoolean("Turret Using Limelight", usingLimelight);
    SmartDashboard.putNumber("Turret Limelight ID", LimelightLib.getFiducialID(LIMELIGHT_NAME));

    boolean atLeftLimit  = currentAngle >= MAX_TURRET;
    boolean atRightLimit = currentAngle <= MIN_TURRET;

    if ((atLeftLimit && output > 0) || (atRightLimit && output < 0)) {
      turret.manualTurret(0);
    } else {
      turret.manualTurret(-output);
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