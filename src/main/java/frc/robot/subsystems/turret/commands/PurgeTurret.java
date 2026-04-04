package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class PurgeTurret extends Command {

    private static final double FEED_PURGE_VOLTAGE     = 4.0;
    private static final double FLYWHEEL_PURGE_VOLTAGE = -4.0;

    private final Turret turret;
    Spindexer spindexer;

    public PurgeTurret(Turret turret, Spindexer spindexer) {
        this.turret = turret;
        this.spindexer = spindexer;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        turret.spinFeed(FEED_PURGE_VOLTAGE);
        turret.flywheelSpark1.set(FLYWHEEL_PURGE_VOLTAGE);
        SmartDashboard.putNumber("Turret State", turret.getFeedVelocity());
        spindexer.spinIndexer(0.2);
        
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