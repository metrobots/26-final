package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class Pass extends Command {

    // -----------------------------------------------------------------------
    // SUBSYSTEMS
    // -----------------------------------------------------------------------

    private final Turret turret;
    private final Spindexer spindexer;
    private final CommandXboxController prim;

    // -----------------------------------------------------------------------
    // FIXED PASS TARGETS
    // -----------------------------------------------------------------------

    private static final double PASS_FLYWHEEL_RPS = -30.0;
    private static final double PASS_HOOD_ANGLE   = 0.0;
    private static final double PASS_TURRET_ANGLE = 0.0;

    // -----------------------------------------------------------------------
    // FLYWHEEL CONSTANTS
    // -----------------------------------------------------------------------

    private static final double FLYWHEEL_TOLERANCE_RPS = 3;

    // -----------------------------------------------------------------------
    // HOOD CONSTANTS
    // -----------------------------------------------------------------------

    private static final double HOOD_TOL = 0.5;

    // -----------------------------------------------------------------------
    // TURRET CONSTANTS
    // -----------------------------------------------------------------------

    private static final double TURRET_AIM_TOL_DEG = 0.5;

    // -----------------------------------------------------------------------
    // FEEDER CONSTANTS
    // -----------------------------------------------------------------------

    private static final double TARGET_FEED_RPS = 190.0;
    private static final double FEED_kS         = 0.15;
    private static final double FEED_kV         = 0.0857;
    private static final double FEED_kA         = 1.2;

    // -----------------------------------------------------------------------
    // CONTROLLERS
    // -----------------------------------------------------------------------

    private final PIDController feedPID   = new PIDController(0.12, 0.008, 0.0);
    private final PIDController hoodPID   = new PIDController(0.18, 0.0,  0.0);
    private final PIDController turretPID = new PIDController(0.03, 0.0,  0.0);

    private final SimpleMotorFeedforward feedFF =
        new SimpleMotorFeedforward(FEED_kS, FEED_kV, FEED_kA);

    // -----------------------------------------------------------------------
    // RPS DROP TRACKING
    // -----------------------------------------------------------------------

    private double maxObservedRPS = Double.NEGATIVE_INFINITY;
    private double biggestRPSDrop = 0.0;

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    public Pass(Turret turret, Spindexer spindexer, CommandXboxController prim) {
        this.turret    = turret;
        this.spindexer = spindexer;
        this.prim      = prim;

        addRequirements(turret, spindexer);

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
        maxObservedRPS = Double.NEGATIVE_INFINITY;
        biggestRPSDrop = 0.0;
    }

    // -----------------------------------------------------------------------
    // EXECUTE
    // -----------------------------------------------------------------------

    @Override
    public void execute() {

        // -------------------------------------------------------------------
        // 1. TURRET — hold center
        // -------------------------------------------------------------------

        double currentAngle = turret.getTurretAngleRelative();

        double turretOutput = MathUtil.clamp(
            turretPID.calculate(currentAngle, PASS_TURRET_ANGLE),
            -0.6, 0.6
        );
        turret.manualTurret(-turretOutput);

        // -------------------------------------------------------------------
        // 2. HOOD — fixed pass angle
        // -------------------------------------------------------------------

        double hoodOutput = hoodPID.calculate(turret.hoodEncoder.getPosition(), PASS_HOOD_ANGLE);
        turret.manualHood(hoodOutput);

        // -------------------------------------------------------------------
        // 3. FLYWHEEL — fixed pass speed
        // -------------------------------------------------------------------

        double currentRPS = turret.getFlywheelVelocity();
        turret.setFlywheelVelocity(PASS_FLYWHEEL_RPS);

        // -------------------------------------------------------------------
        // 4. RPS DROP TRACKING
        // -------------------------------------------------------------------

        double absRPS = Math.abs(currentRPS);
        if (absRPS > maxObservedRPS) {
            maxObservedRPS = absRPS;
        }
        double drop = maxObservedRPS - absRPS;
        if (drop > biggestRPSDrop) {
            biggestRPSDrop = drop;
        }

        // -------------------------------------------------------------------
        // 5. READINESS FLAGS
        // -------------------------------------------------------------------

        boolean atSpeed   = Math.abs(currentRPS - PASS_FLYWHEEL_RPS) <= FLYWHEEL_TOLERANCE_RPS;
        boolean aimed     = Math.abs(currentAngle - PASS_TURRET_ANGLE) <= TURRET_AIM_TOL_DEG;
        boolean hoodReady = Math.abs(turret.hoodEncoder.getPosition() - PASS_HOOD_ANGLE) <= HOOD_TOL;

        prim.getHID().setRumble(RumbleType.kBothRumble, atSpeed ? 0.7 : 0.0);

        // -------------------------------------------------------------------
        // 6. FEEDER + INDEXER
        // -------------------------------------------------------------------

        if (atSpeed) {
            spindexer.spinIndexer(-0.2);

            double feedVoltage = MathUtil.clamp(
                feedFF.calculate(TARGET_FEED_RPS)
                    + feedPID.calculate(turret.getFeedVelocity(), TARGET_FEED_RPS),
                -12.0, 12.0
            );
            turret.feedSpark.setVoltage(feedVoltage);
            SmartDashboard.putNumber("Feed Voltage", feedVoltage);

        } else {
            turret.spinFeed(0.0);
            spindexer.spinIndexer(0.0);
        }

        // -------------------------------------------------------------------
        // 7. DEBUG
        // -------------------------------------------------------------------

        SmartDashboard.putNumber("Turret Angle",    currentAngle);
        SmartDashboard.putNumber("Hood Angle",      turret.hoodEncoder.getPosition());
        SmartDashboard.putNumber("Flywheel RPS",    currentRPS);
        SmartDashboard.putNumber("Feed Velocity",   turret.getFeedVelocity());
        SmartDashboard.putBoolean("Turret Ready",   aimed);
        SmartDashboard.putBoolean("Flywheel Ready", atSpeed);
        SmartDashboard.putBoolean("Hood Ready",     hoodReady);
        SmartDashboard.putBoolean("Ready To Pass",  atSpeed);
        SmartDashboard.putNumber("Biggest RPS Drop", biggestRPSDrop);
    }

    // -----------------------------------------------------------------------
    // END
    // -----------------------------------------------------------------------

    @Override
    public void end(boolean interrupted) {
        turret.manualTurret(0);
        turret.stopFlywheel();
        turret.spinFeed(0);
        spindexer.spinIndexer(0.0);
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