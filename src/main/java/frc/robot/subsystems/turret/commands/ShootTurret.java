package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // 🔁 Target must be NEGATIVE for flywheel direction
    private static final double TARGET_RPM = -33.0;

    // 🎯 Tune these values
    private final PIDController pid = new PIDController(0.0005, 0.0, 0.0);
    private final SimpleMotorFeedforward ff =
            new SimpleMotorFeedforward(0.2, 0.00015);

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);

        pid.setTolerance(2); // ±2 RPM tolerance
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        double currentRPM = turret.getFlywheelRPM();

        SmartDashboard.putNumber("flywheelRPM", currentRPM);
        SmartDashboard.putNumber("feedRPM", turret.getFeedRPM());

        // PID correction
        double pidOutput = pid.calculate(currentRPM, TARGET_RPM);

        // Feedforward (calculate with positive magnitude, apply negative sign)
        double ffOutput = -ff.calculate(Math.abs(TARGET_RPM));

        // Combine
        double output = pidOutput + ffOutput;

        // 🔒 Clamp so it NEVER goes positive (prevents braking) // thank you chatGPT for this AMAZING comment
        output = Math.min(0.0, Math.max(-1.0, output));

        turret.manualFlywheels(output);

        // Only run feeder when at speed
        if (pid.atSetpoint()) {
            turret.spinFeed(0.5);
        } else {
            turret.spinFeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.manualFlywheels(0);
        turret.spinFeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}