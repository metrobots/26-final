package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // Target speed in RPS
    private static final double TARGET_RPS = -30.0;

    // PID constants (small because FF does most work)
    private final PIDController pid = new PIDController(0.7, 0.0, 0.0);

    // Feedforward constants (TUNE THESE)
    private static final double kS = 0.4;    // volts to overcome friction
    private static final double kV = 0.2;    // volts per RPS (starting guess)

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV);

    // Track if flywheel has reached speed once
    private boolean hasReachedSpeed = false;

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);

        pid.setTolerance(1.0); // ±1 RPS tolerance
    }

    @Override
    public void initialize() {
        pid.reset();
        hasReachedSpeed = false;
    }

    @Override
    public void execute() {

        double currentRPS = turret.flywheelSpark1.getEncoder().getVelocity();
        double feedRPS = turret.feedSpark.getEncoder().getVelocity();

        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", TARGET_RPS);

        // Check if we've reached target speed at least once
        if (!hasReachedSpeed && Math.abs(currentRPS - TARGET_RPS) <= 1.0) {
            hasReachedSpeed = true;
        }

        // PID correction (volts)
        double pidOutput = pid.calculate(currentRPS, TARGET_RPS);

        // Feedforward prediction (volts)
        double ffOutput = ff.calculate(TARGET_RPS);

        // Combine PID + FF
        double totalVoltage = pidOutput + ffOutput;

        // ------------------------------
        // RECOVERY MODE (only after reaching speed, only when below target)
        // ------------------------------
        boolean recoveryMode = hasReachedSpeed && currentRPS > (TARGET_RPS + 1.0);

        if (recoveryMode) {
            totalVoltage = -12.0;    // full send recovery
        }

        turret.spinFeed(1);    // normal feed

        SmartDashboard.putBoolean("Recovery Mode", recoveryMode);
        SmartDashboard.putBoolean("Has Reached Speed", hasReachedSpeed);

        // Clamp voltage
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        SmartDashboard.putNumber("PID Volts", pidOutput);
        SmartDashboard.putNumber("FF Volts", ffOutput);
        SmartDashboard.putNumber("Total Voltage", totalVoltage);
        SmartDashboard.putNumber("Feed Velocity", feedRPS);

        turret.setFlywheelVoltage(totalVoltage);
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