package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // Target speed in RPS
    private static final double TARGET_RPS = -20.0;

    // PID constants (small because FF does most work)
    private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

    // Feedforward constants (TUNE THESE)
    private static final double kS = 0.15;    // volts to overcome friction
    private static final double kV = 0.15;    // volts per RPS (starting guess)

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV);

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

        double currentRPS = turret.flywheelSpark1.getEncoder().getVelocity();

        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", TARGET_RPS);

        // PID correction (volts)
        double pidOutput = pid.calculate(currentRPS, TARGET_RPS);

        // Feedforward prediction (volts)
        double ffOutput = ff.calculate(TARGET_RPS);

        // Combine PID + FF
        double totalVoltage = pidOutput + ffOutput;

        // Clamp to real voltage range
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        SmartDashboard.putNumber("PID Volts", pidOutput);
        SmartDashboard.putNumber("FF Volts", ffOutput);
        SmartDashboard.putNumber("Total Voltage", totalVoltage);

        turret.setFlywheelVoltage(totalVoltage);

        turret.spinFeed(0.7);

        // Only feed when at speed
        // if (pid.atSetpoint()) {
        //     turret.spinFeed(-0.7);
        // } else {
        //     turret.spinFeed(0.0);
        // }
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