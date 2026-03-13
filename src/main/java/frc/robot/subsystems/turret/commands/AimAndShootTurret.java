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

    // How many consecutive execute() loops all conditions must hold before feeding.
    // At 20ms per loop, 10 = 200ms of stability required before firing.
    private static final int READY_COUNT_THRESHOLD = 10;
    private int readyCount = 0;

    // Flywheel PID
    // TODO: Replace kP with SysId-characterized value
    private final PIDController flywheelPID = new PIDController(0.5, 0.0, 0.0);

    // Feed PID
    private final PIDController feedPID = new PIDController(0.12, 0.0, 0.0);

    // Hood PID
    private final PIDController hoodPID = new PIDController(0.18, 0, 0);

    // Turret PID
    private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);

    private static final double MAX_TURRET = 30.0;
    private static final double MIN_TURRET = -40.0;

    // Flywheel feedforward
    // TODO: Run SysId to characterize these — kA is unused without acceleration input
    private static final double kS = 0.2;
    private static final double kV = 0.13;

    // Feed motor feedforward
    private static final double feedkS = 0.15;
    private static final double feedkV = 0.12;

    // NOTE: kA removed from both feedforward instances.
    // SimpleMotorFeedforward.calculate(velocity) does not use kA — it requires
    // calculate(velocity, acceleration). Without an acceleration setpoint, kA
    // contributed nothing but noise. Characterize with SysId and pass acceleration
    // if you want kA to have effect.
    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(kS, kV);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(feedkS, feedkV);

    // Feeder soft-start: ramp voltage up over this many loops after gate opens
    // to avoid slamming the first piece into a flywheel that's right at the edge
    // of the tolerance band.
    private static final int FEED_RAMP_LOOPS = 5;
    private int feedRampCount = 0;

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
        readyCount = 0;
        feedRampCount = 0;
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

        double angularVelocity = drivetrain.getTurnRate();
        // TODO: Tune this scalar on the real robot
        double turretVelocityFF = angularVelocity * 0.005;

        double turretOutput = MathUtil.clamp(
            turretPID.calculate(currentAngle, targetAngle) + turretVelocityFF,
            -0.6, 0.6);

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
        READINESS — sustained stability gate
        -------------------------
        All three conditions must hold for READY_COUNT_THRESHOLD consecutive loops.
        This prevents the feeder from firing on a momentary flicker through the
        tolerance band, which was the primary cause of inter-piece inconsistency.
        */

        boolean atSpeed   = Math.abs(currentRPS - targetRPS) <= 2.5;
        boolean aimed     = Math.abs(currentAngle - targetAngle) <= 0.5;
        boolean hoodReady = Math.abs(turret.hoodEncoder.getPosition() - hoodAngle) <= 0.3;

        if (atSpeed && aimed && hoodReady) {
            readyCount = Math.min(readyCount + 1, READY_COUNT_THRESHOLD);
        } else {
            // Any condition dropping out resets the counter entirely.
            // The feeder will cut immediately and the count must rebuild from zero.
            readyCount = 0;
            feedRampCount = 0;
        }

        boolean sustainedReady = readyCount >= READY_COUNT_THRESHOLD;

        /*
        -------------------------
        FEEDER CONTROL
        -------------------------
        */

        if (sustainedReady) {

            // Soft-start: scale voltage from 0 → full over FEED_RAMP_LOOPS loops.
            // This prevents the first ball hitting a flywheel that's right at the
            // edge of atSpeed, giving the flywheel a moment to absorb the load
            // before full feed rate is applied.
            feedRampCount = Math.min(feedRampCount + 1, FEED_RAMP_LOOPS);
            double rampScale = (double) feedRampCount / FEED_RAMP_LOOPS;

            double feedVelocity = turret.getFeedVelocity();
            double feedFFVolts  = feedFF.calculate(TARGET_FEED_RPS);
            double feedPIDVolts = feedPID.calculate(feedVelocity, TARGET_FEED_RPS);

            double feedVoltage =
                MathUtil.clamp((feedFFVolts + feedPIDVolts) * rampScale, -12.0, 12.0);

            turret.spinFeed(feedVoltage);

            SmartDashboard.putNumber("Feed Voltage", feedVoltage);
            SmartDashboard.putNumber("Feed Ramp Scale", rampScale);

        } else {

            turret.spinFeed(0.0);
            SmartDashboard.putNumber("Feed Voltage", 0.0);
            SmartDashboard.putNumber("Feed Ramp Scale", 0.0);
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
        SmartDashboard.putBoolean("Hood Ready", hoodReady);
        SmartDashboard.putBoolean("Sustained Ready", sustainedReady);
        SmartDashboard.putNumber("Ready Count", readyCount);
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