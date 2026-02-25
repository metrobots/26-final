package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // 2000 RPM ≈ 33.3 RPS
    private static final double TARGET_RPS = -33;

    private final PIDController pid = new PIDController(2, 0.0, 0.0);

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
        pid.setTolerance(1.0); // 1 RPS tolerance
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        double currentRPS = turret.getFlywheelVelocity();

        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", TARGET_RPS);

        double pidVolts = pid.calculate(currentRPS, TARGET_RPS);

        SmartDashboard.putNumber("Total Volts", pidVolts);

        turret.setFlywheelVoltage(pidVolts);

        if (Math.abs(currentRPS - TARGET_RPS) < 1.0) {
            turret.spinFeed(0.5);
        } else {
            turret.spinFeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setFlywheelVoltage(0);
        turret.spinFeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}