package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimAndShootTurret extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;

    private final TurretHoodTable table = new TurretHoodTable();

    private static final double TARGET_FEED_RPS = 100.0;

    // Flywheel PID
    private final PIDController flywheelPID = new PIDController(0.35, 0.0, 0.0);

    // Feed PID
    private final PIDController feedPID = new PIDController(0.12, 0.0, 0.0);

    // Hood PID
    private final PIDController hoodPID = new PIDController(0.18, 0, 0);

    // Turret PID
    private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);

    private static final double MAX_TURRET = 40.0;
    private static final double MIN_TURRET = -40.0;

    // Flywheel feedforward
    private static final double kS = 0.2;
    private static final double kV = 0.13;
    private static final double kA = 1.2;

    // Feed motor feedforward
    private static final double feedkS = 0.15;
    private static final double feedkV = 0.12;
    private static final double feedkA = 1.2;

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV, kA);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(feedkS, feedkV, feedkA);

    public AimAndShootTurret(Turret turret, Drivetrain drivetrain) {

        this.turret = turret;
        this.drivetrain = drivetrain;

        addRequirements(turret);

        flywheelPID.setTolerance(1.0);
        turretPID.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        flywheelPID.reset();
        turretPID.reset();
    }

    @Override
    public void execute() {

        /*
        -------------------------
        TURRET AIMING
        -------------------------
        */

        double targetAngle =
            MathUtil.clamp(-drivetrain.getAngleToCenter(), MIN_TURRET, MAX_TURRET);

        double currentAngle = turret.getTurretAngleRelative();

        double turretOutput =
            MathUtil.clamp(turretPID.calculate(currentAngle, targetAngle), -0.6, 0.6);

        boolean atLeftLimit = currentAngle >= MAX_TURRET;
        boolean atRightLimit = currentAngle <= MIN_TURRET;

        if ((atLeftLimit && turretOutput > 0) || (atRightLimit && turretOutput < 0)) {
            turret.manualTurret(0);
        } else {
            turret.manualTurret(-turretOutput);
        }

        /*
        -------------------------
        DISTANCE CALCULATION
        -------------------------
        */

        double distance = drivetrain.getDistanceToCenter();

        TurretHoodTable.HoodData data = table.get(distance);

        double targetRPS = -data.speed;
        double hoodAngle = data.angle;

        /*
        -------------------------
        HOOD CONTROL
        -------------------------
        */

        double hoodOutput =
            hoodPID.calculate(turret.hoodEncoder.getPosition(), hoodAngle);

        turret.manualHood(hoodOutput);

        /*
        -------------------------
        FLYWHEEL CONTROL
        -------------------------
        */

        double currentRPS = turret.getFlywheelVelocity();

        double ffOutput = ff.calculate(targetRPS);
        double pidOutput = flywheelPID.calculate(currentRPS, targetRPS);

        double totalVoltage =
            MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        turret.setFlywheelVoltage(totalVoltage);



        /*
        -------------------------
        FEEDER CONTROL
        -------------------------
        */

        double feedVelocity = turret.getFeedVelocity();

        boolean atSpeed = Math.abs(currentRPS - targetRPS) <= 5.0;
        boolean aimed = (currentAngle >= targetAngle - 0.5) && (currentAngle <= targetAngle + 0.5);

        if (atSpeed) {

            double feedFFVolts = feedFF.calculate(TARGET_FEED_RPS);
            double feedPIDVolts =
                feedPID.calculate(feedVelocity, TARGET_FEED_RPS);

            double feedVoltage =
                MathUtil.clamp(feedFFVolts + feedPIDVolts, -12.0, 12.0);

            turret.spinFeed(feedVoltage);

            SmartDashboard.putNumber("Feed Voltage", feedVoltage);

        } else {

            turret.spinFeed(0.0);
        }

        /*
        -------------------------
        DEBUG VALUES
        -------------------------
        */

        SmartDashboard.putNumber("DistanceToCenter", distance);
        SmartDashboard.putNumber("Turret Target", targetAngle);
        SmartDashboard.putNumber("Turret Angle", currentAngle);
        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", targetRPS);
        SmartDashboard.putNumber("Total Voltage", totalVoltage);
        SmartDashboard.putBoolean("Turret Ready", aimed);
        SmartDashboard.putBoolean("Flywheel Ready", atSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        turret.manualTurret(0);
        turret.setFlywheelVoltage(0);
        turret.spinFeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
