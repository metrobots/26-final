package frc.robot.subsystems.drivetrain.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.OIConstants;

public class DriveToPose extends Command {

    private static final double MIN_SPEED = 0.1;

    private final Drivetrain drivetrain;
    private final Pose2d targetPose;
    private final CommandXboxController controller;

    private Command pathfindCommand;

    /**
     * Pathfinds to targetPose continuously using AutoBuilder.pathfindToPose.
     * Left stick Y scales speed live. Right stick X applies manual rotation
     * on top of the path's translation. Any other input cancels the command.
     */
    public DriveToPose(Drivetrain drivetrain, Pose2d targetPose, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rebuildPathfindCommand();
        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        double speedScale = getSpeedScale();

        // Rebuild and restart the pathfind command each loop with updated constraints
        pathfindCommand.end(true);
        rebuildPathfindCommand(speedScale);
        pathfindCommand.initialize();
        pathfindCommand.execute();

        // Overwrite omega with manual right stick X rotation
        double rot = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband)
                * AutoConstants.kMaxAngularSpeedRadiansPerSecond;

        if (Math.abs(rot) > 0) {
            ChassisSpeeds speeds = drivetrain.getRobotRelativeSpeeds();
            drivetrain.driveRobotRelative(new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    rot
            ));
        }
    }

    @Override
    public boolean isFinished() {
        if (hasCancelInput()) return true;
        return pathfindCommand != null && pathfindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathfindCommand != null) pathfindCommand.end(interrupted);
    }

    // ── Helpers ────────────────────────────────────────────────────────────────

    private void rebuildPathfindCommand() {
        rebuildPathfindCommand(getSpeedScale());
    }

    private void rebuildPathfindCommand(double speedScale) {
        PathConstraints constraints = new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond               * speedScale,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared * speedScale,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
        );

        pathfindCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // goal end velocity
        );
    }

    /**
     * Maps left stick Y (forward = positive after negation) to MIN_SPEED–1.0.
     * If the stick is fully released, MIN_SPEED keeps the robot crawling forward.
     */
    private double getSpeedScale() {
        double raw   = -controller.getLeftY();
        double input = MathUtil.applyDeadband(raw, OIConstants.kDriveDeadband);
        input = MathUtil.clamp(input, 0.0, 1.0);
        return MathUtil.interpolate(MIN_SPEED, 1.0, input);
    }

    /**
     * Returns true if any input that should cancel the path is detected.
     * Left stick Y and right stick X are the only allowed inputs.
     */
    private boolean hasCancelInput() {
        double db = OIConstants.kDriveDeadband;

        boolean leftStickX =
                Math.abs(MathUtil.applyDeadband(controller.getLeftX(), db)) > 0;

        boolean triggers =
                controller.getLeftTriggerAxis()  > db ||
                controller.getRightTriggerAxis() > db;

        boolean buttons =
                controller.a().getAsBoolean()           ||
                controller.b().getAsBoolean()           ||
                controller.x().getAsBoolean()           ||
                controller.y().getAsBoolean()           ||
                controller.leftBumper().getAsBoolean()  ||
                controller.rightBumper().getAsBoolean() ||
                controller.start().getAsBoolean()       ||
                controller.back().getAsBoolean()        ||
                controller.povUp().getAsBoolean()       ||
                controller.povDown().getAsBoolean()     ||
                controller.povLeft().getAsBoolean()     ||
                controller.povRight().getAsBoolean();

        return leftStickX || triggers || buttons;
    }
}