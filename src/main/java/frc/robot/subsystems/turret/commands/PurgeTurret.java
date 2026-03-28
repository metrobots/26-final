package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class PurgeTurret extends Command {

    private static final double FEED_PURGE_VOLTAGE     = 4.0;
    private static final double FLYWHEEL_PURGE_VOLTAGE = -4.0;

    private final Turret turret;

    public PurgeTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        turret.spinFeed(FEED_PURGE_VOLTAGE);
        turret.flywheelSpark1.set(FLYWHEEL_PURGE_VOLTAGE);
    }

    @Override
    public void end(boolean interrupted) {
        turret.spinFeed(0);
        turret.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button is released
    }
}