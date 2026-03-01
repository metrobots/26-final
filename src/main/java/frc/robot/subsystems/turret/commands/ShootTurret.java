package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // 2000 RPM ≈ 33.3 RPS
    private static final double TARGETRPM = -20;

    private final PIDController pid = new PIDController(0.5, 0.0, 0.0);

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

        double currentRPM = turret.flywheelSpark1.getEncoder().getVelocity();

        SmartDashboard.putNumber("Flywheel RPS", currentRPM);
        SmartDashboard.putNumber("Target RPS", TARGETRPM);

        double output = pid.calculate(currentRPM, TARGETRPM);

        output = Math.max(-1.0, Math.min(1.0, output));

        SmartDashboard.putNumber("output", output);

        turret.setFlywheelVoltage(output);

        if (Math.abs(currentRPM - TARGETRPM) < 3.0) {
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