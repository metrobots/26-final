package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.spindexer.Spindexer;

public class AimAndShootTurret extends Command {

    // -----------------------------------------------------------------------
    // SUBSYSTEMS
    // -----------------------------------------------------------------------

    private final Turret turret;
    private final Drivetrain drivetrain;
    private final CommandXboxController prim;
    private final Spindexer indexer;

    private final TurretHoodTable table = new TurretHoodTable();

    // -----------------------------------------------------------------------
    // TURRET CONSTANTS
    // -----------------------------------------------------------------------

    private static final double MAX_TURRET_DEG      = 40.0;
    private static final double MIN_TURRET_DEG      = -40.0;
    private static final double TURRET_AIM_TOL_DEG  = 0.5;   // degrees — aimed check
    private static final double TURRET_ANGLE_OFFSET  = 3.0;   // degrees — systematic bias correction
    private static final double TURRET_VEL_FF_SCALAR = 0.005; // counteracts drivetrain rotation lag — TUNE

    // -----------------------------------------------------------------------
    // FLYWHEEL CONSTANTS
    // -----------------------------------------------------------------------

    private static final double FLYWHEEL_TOLERANCE_RPS = 1.0; // tighten to 0.75 once gains are dialled — TUNE

    // -----------------------------------------------------------------------
    // HOOD CONSTANTS
    // -----------------------------------------------------------------------

    private static final double HOOD_TOL = 0.5; // encoder units

    // -----------------------------------------------------------------------
    // FEEDER CONSTANTS
    // -----------------------------------------------------------------------

    private static final double TARGET_FEED_RPS = 110.0;
    private static final double FEED_kS         = 0.15;
    private static final double FEED_kV         = 0.0857;
    private static final double FEED_kA         = 1.2;

    // -----------------------------------------------------------------------
    // SHOOT-ON-THE-MOVE CONSTANTS
    // -----------------------------------------------------------------------

    private static final double FLIGHT_TIME_PER_METER = 0.055; // seconds/meter — TUNE
    private static final double LEAD_SCALAR           = 0.5;   // 0.8–1.2 — TUNE

    // ---------------------------------------
    // --------------------------------
    // PULSE CONSTANTS
    // -----------------------------------------------------------------------

    /** How long the feed/indexer run per ball. Shorten if double-feeding. */
    private static final double PULSE_ON_DURATION  = 0.25;  // seconds — TUNE

    /** Gap between pulses — lets ball clear and flywheel recover speed. */
    private static final double PULSE_OFF_DURATION = 0.15; // seconds — TUNE

    // -----------------------------------------------------------------------
    // CONTROLLERS
    // -----------------------------------------------------------------------

    private final PIDController feedPID   = new PIDController(0.12, 0.0, 0.0);
    private final PIDController hoodPID   = new PIDController(0.18, 0.0, 0.0);
    private final PIDController turretPID = new PIDController(0.01, 0.0, 0.0);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(FEED_kS, FEED_kV, FEED_kA);

    // -----------------------------------------------------------------------
    // PULSE STATE
    // -----------------------------------------------------------------------

    private final Timer pulseTimer = new Timer();
    private boolean pulseOn = false;

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    public AimAndShootTurret(
            Turret turret,
            Drivetrain drivetrain,
            CommandXboxController prim,
            Spindexer indexer) {

        this.turret     = turret;
        this.drivetrain = drivetrain;
        this.prim       = prim;
        this.indexer    = indexer;

        addRequirements(turret, indexer);

        turretPID.setTolerance(TURRET_AIM_TOL_DEG);
    }

    // -----------------------------------------------------------------------
    // INITIALIZE
    // -----------------------------------------------------------------------

    @Override
    public void initialize() {
        feedPID.reset();
        hoodPID.reset();
        turretPID.reset();

        // Start timer in the OFF phase so the first pulse fires immediately
        // once the robot becomes ready to shoot.
        pulseOn = false;
        pulseTimer.restart();
    }

    // -----------------------------------------------------------------------
    // EXECUTE
    // -----------------------------------------------------------------------

    @Override
    public void execute() {

        // -------------------------------------------------------------------
        // 1. DISTANCE + LOOKUP
        // -------------------------------------------------------------------

        double distance = drivetrain.getDistanceToCenter();
        TurretHoodTable.HoodData data = table.get(distance);

        double targetRPS = -data.speed;
        double hoodAngle =  data.angle;

        // -------------------------------------------------------------------
        // 2. TURRET AIMING (with shoot-on-the-move lead)
        // -------------------------------------------------------------------

        double lateralVelocity = drivetrain.getLateralVelocityToTarget();
        double flightTime      = distance * FLIGHT_TIME_PER_METER;

        double leadAngle = Math.toDegrees(
            Math.atan2(lateralVelocity * flightTime, distance)
        ) * LEAD_SCALAR;

        double targetAngle = MathUtil.clamp(
            -drivetrain.getAngleToCenter() + leadAngle + TURRET_ANGLE_OFFSET,
            MIN_TURRET_DEG,
            MAX_TURRET_DEG
        );

        double currentAngle     = turret.getTurretAngleRelative();
        double turretVelocityFF = drivetrain.getTurnRate() * TURRET_VEL_FF_SCALAR;

        double turretOutput = MathUtil.clamp(
            turretPID.calculate(currentAngle, targetAngle) + turretVelocityFF,
            -0.6, 0.6
        );

        boolean atLeftLimit  = currentAngle >= MAX_TURRET_DEG;
        boolean atRightLimit = currentAngle <= MIN_TURRET_DEG;

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
        // 4. FLYWHEEL CONTROL (onboard PID on the SPARK Flex)
        //
        //    setFlywheelVelocity() writes the setpoint to the SPARK Flex,
        //    which runs its own closed-loop at ~1000 Hz internally.
        // -------------------------------------------------------------------

        double currentRPS = turret.getFlywheelVelocity();
        turret.setFlywheelVelocity(targetRPS);

        // -------------------------------------------------------------------
        // 5. READINESS FLAGS
        // -------------------------------------------------------------------

        boolean atSpeed      = Math.abs(currentRPS - targetRPS) <= FLYWHEEL_TOLERANCE_RPS;
        boolean aimed        = Math.abs(currentAngle - targetAngle) <= TURRET_AIM_TOL_DEG;
        boolean hoodReady    = Math.abs(turret.hoodEncoder.getPosition() - hoodAngle) <= HOOD_TOL;
        boolean readyToShoot = atSpeed;

        prim.getHID().setRumble(RumbleType.kBothRumble, readyToShoot ? 0.7 : 0.0);

        // -------------------------------------------------------------------
        // 6. PULSED FEEDER + INDEXER
        //
        //    Cycles between PULSE_ON_DURATION (feed fires) and
        //    PULSE_OFF_DURATION (feed pauses) to prevent double-feeding.
        //    Any loss of readiness immediately cuts the feed and resets the
        //    pulse state so the next ball starts from a clean ON pulse.
        // -------------------------------------------------------------------

        if (readyToShoot) {
            // Advance pulse state machine
            if (!pulseOn && pulseTimer.hasElapsed(PULSE_OFF_DURATION)) {
                pulseOn = true;
                pulseTimer.restart();
            } else if (pulseOn && pulseTimer.hasElapsed(PULSE_ON_DURATION)) {
                pulseOn = false;
                pulseTimer.restart();
            }

            indexer.spinIndexer(-0.08);

            if (pulseOn) {
                double feedVoltage = MathUtil.clamp(
                    feedFF.calculate(TARGET_FEED_RPS)
                        + feedPID.calculate(turret.getFeedVelocity(), TARGET_FEED_RPS),
                    -12.0, 12.0
                );
                turret.feedSpark.setVoltage(feedVoltage);
                SmartDashboard.putNumber("Feed Voltage", feedVoltage);
            } else {
                turret.spinFeed(0.0);
            }

        } else {
            // Not ready — cut feed immediately and reset so the next ball
            // starts with a full ON pulse rather than mid-cycle.
            pulseOn = false;
            pulseTimer.restart();
            turret.spinFeed(0.0);
            indexer.spinIndexer(0.0);
        }

        // -------------------------------------------------------------------
        // 7. DEBUG
        // -------------------------------------------------------------------

        SmartDashboard.putNumber("Distance To Center",  distance);
        SmartDashboard.putNumber("Hood Angle",          turret.hoodEncoder.getPosition());
        SmartDashboard.putNumber("Lateral Velocity",    lateralVelocity);
        SmartDashboard.putNumber("Lead Angle (deg)",    leadAngle);
        SmartDashboard.putNumber("Turret Target",       targetAngle);
        SmartDashboard.putNumber("Turret Angle",        currentAngle);
        SmartDashboard.putNumber("Flywheel RPS",        currentRPS);
        SmartDashboard.putNumber("Target RPS",          targetRPS);
        SmartDashboard.putNumber("Feed Velocity",       turret.getFeedVelocity());
        SmartDashboard.putNumber("Indexer Current",     indexer.getIndexerCurrent());
        SmartDashboard.putBoolean("Turret Ready",       aimed);
        SmartDashboard.putBoolean("Flywheel Ready",     atSpeed);
        SmartDashboard.putBoolean("Hood Ready",         hoodReady);
        SmartDashboard.putBoolean("Ready To Shoot",     readyToShoot);
        SmartDashboard.putBoolean("Pulse On",           pulseOn);
    }

    // -----------------------------------------------------------------------
    // END
    // -----------------------------------------------------------------------

    @Override
    public void end(boolean interrupted) {
        turret.manualTurret(0);
        turret.stopFlywheel();
        turret.spinFeed(0);
        indexer.spinIndexer(0.0);
        prim.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // -----------------------------------------------------------------------
    // IS FINISHED
    // -----------------------------------------------------------------------

    @Override
    public boolean isFinished() {
        return false;
    }
}