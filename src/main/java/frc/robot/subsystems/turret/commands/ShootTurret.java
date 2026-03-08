package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ShootTurret extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;

    private final TurretHoodTable table = new TurretHoodTable();

    private static final double TARGET_FEED_RPS = 100.0;

    private final PIDController pid = new PIDController(0.35, 0.0, 0.0);
    private final PIDController feedPID = new PIDController(0.12, 0.0, 0.0);

    private static final double kS = 0.2;
    private static final double kV = 0.13;
    private static final double kA = 1.2;

    private static final double feedkS = 0.15;
    private static final double feedkV = 0.12;
    private static final double feedkA = 1.2;

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV, kA);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(feedkS, feedkV, feedkA);

    public ShootTurret(Turret turret, Drivetrain drivetrain) {

        this.turret = turret;
        this.drivetrain = drivetrain;

        addRequirements(turret);

        pid.setTolerance(1.0);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        // Get distance from drivetrain pose estimator
        double distance = drivetrain.getDistanceToCenter();

        // Lookup hood + shooter speed
        TurretHoodTable.HoodData data = table.get(distance);

        double targetRPS = -data.speed;
        double hoodAngle = data.angle;

        // Move hood automatically
        turret.setHoodAngle(hoodAngle);

        double currentRPS = turret.getFlywheelVelocity();

        // Flywheel control
        double ffOutput = ff.calculate(targetRPS);
        double pidOutput = pid.calculate(currentRPS, targetRPS);

        double totalVoltage = pidOutput + ffOutput;
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        turret.setFlywheelVoltage(totalVoltage);

        double feedVelocity = turret.getFeedVelocity();

        if (Math.abs(currentRPS - targetRPS) <= 5.0) {

            double feedFFVolts = feedFF.calculate(TARGET_FEED_RPS);
            double feedPIDVolts = feedPID.calculate(feedVelocity, TARGET_FEED_RPS);

            double feedVoltage = feedFFVolts + feedPIDVolts;
            feedVoltage = Math.max(-12.0, Math.min(12.0, feedVoltage));

            turret.spinFeed(feedVoltage);

            SmartDashboard.putNumber("Feed Voltage", feedVoltage);

        } else {
            turret.spinFeed(0.0);
        }

        // Debug values
        SmartDashboard.putNumber("DistanceToCenter", distance);
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", targetRPS);
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