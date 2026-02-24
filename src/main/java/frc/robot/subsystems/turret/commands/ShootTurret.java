package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {
    private final Turret turret;
    private static final double TARGET_RPM = 2000.0;

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setFlywheelRPM(TARGET_RPM);
    }

    @Override
    public void execute() {
        // turret.setFlywheelRPM(TARGET_RPM);

        // double currentRPM = turret.getFlywheelRPM();
        // boolean atSpeed = Math.abs(currentRPM - TARGET_RPM) < Turret.kFlywheelAtSpeedThresholdRPM;

        // turret.spinFeed(atSpeed ? FEED_SPEED : 0);

        // SmartDashboard.putNumber("Flywheel RPM", currentRPM);
        // SmartDashboard.putNumber("Flywheel Target RPM", TARGET_RPM);
        // SmartDashboard.putBoolean("Flywheel At Speed", atSpeed);
        // SmartDashboard.putNumber("Feeder RPM", turret.getFeedRPM());
        SmartDashboard.putNumber("flywheelRPM", turret.getFlywheelRPM());
        SmartDashboard.putNumber("feedRPM", turret.getFeedRPM());
        turret.manualFlywheels(-0.5);
        turret.spinFeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        turret.manualFlywheels(0);
        turret.spinFeed(0);
        // turret.spinFeed(0);
        // turret.setFlywheelRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
