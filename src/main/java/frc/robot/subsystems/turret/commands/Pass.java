// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pass extends Command {
  /** Creates a new Pass. */

  private static final double TARGET_FEED_RPS = 110.0;
  private static final double FEED_kS         = 0.15;
  private static final double FEED_kV         = 0.0857;
  private static final double FEED_kA         = 1.2;

  private final PIDController feedPID   = new PIDController(0.12, 0.008, 0.0);
  private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);
  private final PIDController hoodPID   = new PIDController(0.18, 0.0, 0.0);
  private final SimpleMotorFeedforward feedFF = new SimpleMotorFeedforward(FEED_kS, FEED_kV, FEED_kA);

  Turret turret;
  Spindexer spindexer;
  public Pass(Turret turret, Spindexer spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.spindexer = spindexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double currentAngle = turret.getTurretAngleRelative();

      double turretOutput = MathUtil.clamp(
          turretPID.calculate(currentAngle, 0),
          -0.6, 0.6
      );

      double feedVoltage = MathUtil.clamp(
                feedFF.calculate(TARGET_FEED_RPS)
                    + feedPID.calculate(turret.getFeedVelocity(), TARGET_FEED_RPS),
                -12.0, 12.0
            );
            turret.feedSpark.setVoltage(feedVoltage);

      double hoodOutput = hoodPID.calculate(turret.hoodEncoder.getPosition(), 2);

      turret.manualTurret(-turretOutput);
      turret.manualHood(hoodOutput);
      turret.setFlywheelVelocity(30);

      turret.spinFeed(0.0);
      spindexer.spinIndexer(-0.08);
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
