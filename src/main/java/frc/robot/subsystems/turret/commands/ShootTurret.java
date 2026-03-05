package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // Target speed in RPS (negative is forward)
    private static final double TARGET_RPS = -85.0; // ~5100 RPM for 5 lb flywheel

    // PID constants for velocity loop
    private final PIDController pid = new PIDController(0.05, 0.0, 0.0);

    // Feedforward constants (tuned for NEO Vortex + 5lb flywheel)
    private static final double kS = 0.2;    // volts to overcome friction
    private static final double kV = 0.3;    // volts per RPS
    private static final double kA = 0.0;    // optional acceleration term

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV, kA);

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);

        pid.setTolerance(1.0); // ±1 RPS tolerance
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        double currentRPS = turret.getFlywheelVelocity();

        // Feedforward voltage
        double ffOutput = ff.calculate(TARGET_RPS);

        // PID correction
        double pidOutput = pid.calculate(currentRPS, TARGET_RPS);

        // Combine PID + FF
        double totalVoltage = pidOutput + ffOutput;

        // Clamp voltage to motor limits
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        turret.setFlywheelVoltage(totalVoltage);

        // Feed only if flywheel is within 2 RPS of target
        if (Math.abs(currentRPS - TARGET_RPS) <= 2.0) {
            turret.spinFeed(1.0);
        } else {
            turret.spinFeed(0.0);
        }

        // SmartDashboard telemetry
        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", TARGET_RPS);
        SmartDashboard.putNumber("PID Volts", pidOutput);
        SmartDashboard.putNumber("FF Volts", ffOutput);
        SmartDashboard.putNumber("Total Voltage", totalVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setFlywheelVoltage(0.0);
        turret.spinFeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}