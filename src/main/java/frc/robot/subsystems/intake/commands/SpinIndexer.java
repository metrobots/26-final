// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IndexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.AimAndShootTurret;

public class SpinIndexer extends Command {

  private final Intake intake;
  private final Turret turret;
  private IndexerState pastState = IndexerState.ACTIVE;

  /**
   * Creates a new SpinIndexer.
   * Spins the indexer only when AimAndShootTurret reports sustained readiness,
   * so balls are not fed into the flywheel during recovery between shots.
   */
  public SpinIndexer(Intake intake, Turret turret) {
    this.intake = intake;
    this.turret = turret;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    this.pastState = intake.currentState;
  }

  @Override
  public void execute() {
      intake.spinIndexer(-0.06);
      turret.feedSpark.set(0.5);
      // intake.spinIndexer(0.0);
  }

  @Override
  public void end(boolean interrupted) {
    intake.spinIndexer(0.0);
    turret.feedSpark.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}