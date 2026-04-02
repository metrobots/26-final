// package frc.robot.subsystems.drivetrain.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.drivetrain.Drivetrain;
// import frc.robot.utils.Constants.AutoConstants;
// import frc.robot.utils.Constants.OIConstants;

// public class DriveToPose extends Command {

//     private static final double MIN_SPEED = 0.1;
//     private static final double SPEED_CHANGE_THRESHOLD = 0.05;

//     private final Drivetrain drivetrain;
//     private final Pose2d targetPose;
//     private final CommandXboxController controller;

//     private Command pathfindCommand;
//     private double lastSpeedScale = -1;

//     /**
//      * Pathfinds to targetPose using AutoBuilder.pathfindToPose.
//      * Right stick Y scales speed live. Any other input cancels the command.
//      */
//     public DriveToPose(Drivetrain drivetrain, Pose2d targetPose, CommandXboxController controller) {
//         this.drivetrain = drivetrain;
//         this.targetPose = targetPose;
//         this.controller = controller;
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
//         lastSpeedScale = -1;
//         double speedScale = getSpeedScale();
//         buildPathfindCommand(speedScale);
//         lastSpeedScale = speedScale;
//         pathfindCommand.initialize();
//     }

//     @Override
//     public void execute() {
//         double speedScale = getSpeedScale();

//         if (Math.abs(speedScale - lastSpeedScale) > SPEED_CHANGE_THRESHOLD) {
//             pathfindCommand.end(true);
//             buildPathfindCommand(speedScale);
//             pathfindCommand.initialize();
//             lastSpeedScale = speedScale;
//         }

//         pathfindCommand.execute();
//     }

//     @Override
//     public boolean isFinished() {
//         if (hasCancelInput()) return true;
//         return pathfindCommand != null && pathfindCommand.isFinished();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (pathfindCommand != null) pathfindCommand.end(interrupted);
//     }

//     // ── Helpers ────────────────────────────────────────────────────────────────

//     private void buildPathfindCommand(double speedScale) {
//         PathConstraints constraints = new PathConstraints(
//                 AutoConstants.kMaxSpeedMetersPerSecond               * speedScale,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared * speedScale,
//                 AutoConstants.kMaxAngularSpeedRadiansPerSecond,
//                 AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
//         );

//         pathfindCommand = AutoBuilder.pathfindToPose(
//                 targetPose,
//                 constraints,
//                 0.0
//         );
//     }

//     /**
//      * Maps right stick Y (forward = positive after negation) to MIN_SPEED–1.0.
//      * If the stick is fully released, MIN_SPEED keeps the robot crawling forward.
//      */
//     private double getSpeedScale() {
//         double raw   = -controller.getRightY();
//         double input = MathUtil.applyDeadband(raw, OIConstants.kDriveDeadband);
//         input = MathUtil.clamp(input, 0.0, 1.0);
//         return MathUtil.interpolate(MIN_SPEED, 1.0, input);
//     }

//     /**
//      * Returns true if any input that should cancel the path is detected.
//      * Right stick Y is the only allowed input.
//      */
//     private boolean hasCancelInput() {
//         double db = OIConstants.kDriveDeadband;

//         boolean leftStick =
//                 Math.abs(MathUtil.applyDeadband(controller.getLeftX(), db)) > 0 ||
//                 Math.abs(MathUtil.applyDeadband(controller.getLeftY(), db)) > 0;

//         boolean rightStickX =
//                 Math.abs(MathUtil.applyDeadband(controller.getRightX(), db)) > 0;

//         boolean triggers =
//                 controller.getLeftTriggerAxis()  > db ||
//                 controller.getRightTriggerAxis() > db;

//         boolean buttons =
//                 controller.a().getAsBoolean()           ||
//                 controller.b().getAsBoolean()           ||
//                 controller.x().getAsBoolean()           ||
//                 controller.y().getAsBoolean()           ||
//                 controller.leftBumper().getAsBoolean()  ||
//                 controller.rightBumper().getAsBoolean() ||
//                 controller.start().getAsBoolean()       ||
//                 controller.back().getAsBoolean()        ||
//                 controller.povUp().getAsBoolean()       ||
//                 controller.povDown().getAsBoolean()     ||
//                 controller.povLeft().getAsBoolean()     ||
//                 controller.povRight().getAsBoolean();

//         return leftStick || rightStickX || triggers || buttons;
//     }
// }