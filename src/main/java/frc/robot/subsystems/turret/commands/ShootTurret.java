package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // Only start feeding when flywheel is within this RPM of target
    private static final double FEED_START_THRESHOLD_RPM = 50.0;
    private static final double FEED_SPEED = 1;
    private boolean feeding = false;

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        feeding = false;
        // // Start flywheel at the target set by AimTurret
        // turret.setFlywheelRPM(turret.getTargetFlywheelRPM());
    }

    @Override
    public void execute() {
        double currentRPM = turret.getFlywheelRPM();
        // double targetRPM = -2000;
        // PIDController feedPID = new PIDController(0.5, 0.0, 0.0); // Tune these values
        // SmartDashboard.putNumber("speed", currentRPM);

        // double feedOutput = feedPID.calculate(currentRPM, targetRPM);
        // feedOutput = MathUtil.clamp(feedOutput, -1, 0);
        turret.manualFlywheels(-0.5);
        SmartDashboard.putNumber("flywheelspeed", currentRPM);

        SmartDashboard.putNumber("feederspeed", turret.getFeedRPM());
        turret.spinFeed(FEED_SPEED);

    }

    @Override
    public void end(boolean interrupted) {
        turret.spinFeed(0);       // Stop feeding
        turret.setFlywheelRPM(0); // Stop flywheel
    }

    @Override
    public boolean isFinished() {
        return false; // Run until canceled externally
    }
}
