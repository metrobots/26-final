package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.spindexer.Spindexer;

public class AimAndShootTurret extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;
    private final CommandXboxController prim;
    private final Spindexer indexer;

    private final TurretHoodTable table = new TurretHoodTable();

    // -----------------------------------------------------------------------
    // CONSTANTS
    // -----------------------------------------------------------------------

    private static final double TARGET_FEED_RPS = 140.0;

    private static final double MAX_TURRET = 40.0;
    private static final double MIN_TURRET = -40.0;

    // Feed feedforward (software — feed motor stays on RoboRIO PID)
    private static final double FEED_kS = 0.15;
    private static final double FEED_kV = 0.0857;
    private static final double FEED_kA = 1.2;

    // Shoot-on-the-move
    private static final double FLIGHT_TIME_PER_METER = 0.055; // seconds/meter — TUNE
    private static final double LEAD_SCALAR           = 0.5;   // 0.8–1.2 — TUNE

    // Turret velocity feedforward scalar (counteracts drivetrain rotation lag)
    private static final double TURRET_VEL_FF_SCALAR = 0.005; // TUNE

    // Turret angle offset to compensate for systematic aiming error
    private static final double TURRET_ANGLE_OFFSET = 1.0; // degrees

    // Flywheel at-speed threshold (RPS). Onboard PID holds tighter so
    // you can tighten this from 2.5 → 1.5 once gains are dialled in.
    private static final double FLYWHEEL_TOLERANCE_RPS = 2.5; // TUNE

    // -----------------------------------------------------------------------
    // CONTROLLERS  (flywheel PID is now onboard — removed from here)
    // -----------------------------------------------------------------------

    private final PIDController feedPID   = new PIDController(0.12, 0.0, 0.0);
    private final PIDController hoodPID   = new PIDController(0.18, 0.0, 0.0);
    private final PIDController turretPID = new PIDController(0.03, 0.0, 0.0);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(FEED_kS, FEED_kV, FEED_kA);

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    public AimAndShootTurret(Turret turret, Drivetrain drivetrain, CommandXboxController prim, Spindexer indexer) {
        this.turret     = turret;
        this.drivetrain = drivetrain;
        this.prim       = prim;
        this.indexer     = indexer;

        addRequirements(turret, indexer);

        turretPID.setTolerance(0.5);
    }

    // -----------------------------------------------------------------------
    // COMMAND LIFECYCLE
    // -----------------------------------------------------------------------

    @Override
    public void initialize() {
        feedPID.reset();
        hoodPID.reset();
        turretPID.reset();
    }

    @Override
    public void execute() {

        // -------------------------------------------------------------------
        // 1. DISTANCE + HOOD/SPEED LOOKUP
        // -------------------------------------------------------------------

        double distance = drivetrain.getDistanceToCenter();
        TurretHoodTable.HoodData data = table.get(distance);

        double targetRPS = -data.speed;
        double hoodAngle = data.angle;

        // -------------------------------------------------------------------
        // 2. TURRET AIMING  (with shoot-on-the-move lead)
        // -------------------------------------------------------------------

        double lateralVelocity = drivetrain.getLateralVelocityToTarget();

        double flightTime = distance * FLIGHT_TIME_PER_METER;

        double leadAngle = Math.toDegrees(
            Math.atan2(lateralVelocity * flightTime, distance)
        ) * LEAD_SCALAR;

        double targetAngle = MathUtil.clamp(
            -drivetrain.getAngleToCenter() + leadAngle + TURRET_ANGLE_OFFSET,
            MIN_TURRET,
            MAX_TURRET
        );

        double currentAngle = turret.getTurretAngleRelative();

        double turretVelocityFF = drivetrain.getTurnRate() * TURRET_VEL_FF_SCALAR;

        double turretOutput = MathUtil.clamp(
            turretPID.calculate(currentAngle, targetAngle) + turretVelocityFF,
            -0.6, 0.6
        );

        boolean atLeftLimit  = currentAngle >= MAX_TURRET;
        boolean atRightLimit = currentAngle <= MIN_TURRET;

        if ((atLeftLimit && turretOutput > 0) || (atRightLimit && turretOutput < 0)) {
            turret.manualTurret(0);
        } else {
            turret.manualTurret(-turretOutput);
        }

        // -------------------------------------------------------------------
        // 3. HOOD CONTROL
        // -------------------------------------------------------------------

        double hoodOutput = hoodPID.calculate(turret.hoodEncoder.getPosition(), hoodAngle);
        turret.manualHood(hoodOutput);

        // -------------------------------------------------------------------
        // 4. FLYWHEEL CONTROL  (onboard PID on the SPARK Flex)
        //
        //    One call per loop is all that's needed. The SPARK Flex runs
        //    its velocity PID at ~1000 Hz internally using the kP/kFF
        //    configured in Turret. No voltage math needed here.
        // -------------------------------------------------------------------

        double currentRPS = turret.getFlywheelVelocity();
        turret.setFlywheelVelocity(targetRPS);

        // -------------------------------------------------------------------
        // 5. FEEDER + INDEXER CONTROL  (only fires when all systems are ready)
        // -------------------------------------------------------------------

        boolean atSpeed      = Math.abs(currentRPS - targetRPS) <= FLYWHEEL_TOLERANCE_RPS;
        boolean aimed        = Math.abs(currentAngle - targetAngle) <= 0.5;
        boolean hoodReady    = Math.abs(turret.hoodEncoder.getPosition() - hoodAngle) <= 0.3;
        boolean readyToShoot = atSpeed && hoodReady;

        prim.getHID().setRumble(RumbleType.kBothRumble, readyToShoot ? 0.7 : 0.0);

        if (readyToShoot) {
            double feedVoltage = MathUtil.clamp(
                feedFF.calculate(TARGET_FEED_RPS)
                    + feedPID.calculate(turret.getFeedVelocity(), TARGET_FEED_RPS),
                -12.0, 12.0
            );
            turret.feedSpark.setVoltage(feedVoltage);
            indexer.spinIndexer(-0.08);
            SmartDashboard.putNumber("Feed Voltage", feedVoltage);
        } else {
            turret.spinFeed(0.0);
            indexer.spinIndexer(0.0);
        }

        // -------------------------------------------------------------------
        // 6. DEBUG
        // -------------------------------------------------------------------

        SmartDashboard.putNumber("Distance To Center", distance);
        SmartDashboard.putNumber("hood angle", turret.hoodEncoder.getPosition());
        SmartDashboard.putNumber("Lateral Velocity",   lateralVelocity);
        SmartDashboard.putNumber("Lead Angle (deg)",   leadAngle);
        SmartDashboard.putNumber("Turret Target",      targetAngle);
        SmartDashboard.putNumber("Turret Angle",       currentAngle);
        SmartDashboard.putNumber("Flywheel RPS",       currentRPS);
        SmartDashboard.putNumber("Target RPS",         targetRPS);
        SmartDashboard.putBoolean("Turret Ready",      aimed);
        SmartDashboard.putBoolean("Flywheel Ready",    atSpeed);
        SmartDashboard.putBoolean("Hood Ready",        hoodReady);
        SmartDashboard.putBoolean("Ready To Shoot",    readyToShoot);
        SmartDashboard.putNumber("feed Velocity",     turret.getFeedVelocity());
        SmartDashboard.putNumber("Indexer Current",     indexer.getIndexerCurrent());
    }

    @Override
    public void end(boolean interrupted) {
        turret.manualTurret(0);
        turret.stopFlywheel();
        turret.spinFeed(0);
        indexer.spinIndexer(0.0);
        prim.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}