package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class PurgeShooter extends Command {

    private final Turret turret;

    public PurgeShooter(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Nothing special to do on start
    }

    @Override
    public void execute() {
        // Continuously aim turret using tx
        turret.aimTurretWithTx();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop turret when command ends
        turret.manualTurret(0);
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (button released)
        return false;
    }
}
