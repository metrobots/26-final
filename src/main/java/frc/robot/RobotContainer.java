package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.Dashboard;
// import frc.robot.subsystems.drivetrain.Drivetrain;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.commands.IntakeDown;
// import frc.robot.subsystems.intake.commands.IntakeIn;
// import frc.robot.subsystems.intake.commands.ShakeIntake;
// import frc.robot.subsystems.turret.Turret;
// import frc.robot.subsystems.turret.commands.AimAndShootTurret;
// import frc.robot.subsystems.turret.commands.HoldZero;
// import frc.robot.subsystems.turret.commands.HoodTarget;
// import frc.robot.subsystems.turret.commands.PurgeTurret;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.commands.MoveSnake;
// import frc.robot.subsystems.spindexer.Spindexer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Auto SendableChooser
  // private final SendableChooser<Command> autoChooser;
  // Subsystem declarations
  // final Dashboard m_dashboard;
  // final Drivetrain m_drivetrain;
  // final Turret m_turret;
  // final Intake m_intake;
  final Lights m_lights;
  // final Spindexer m_spindexer;
  
  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

   public RobotContainer() {

    // m_turret = new Turret();
    // m_drivetrain = new Drivetrain(m_turret);
    // m_dashboard = new Dashboard();
    // m_intake = new Intake(m_dashboard);
    m_lights = new Lights();
    // m_spindexer = new Spindexer();
    
    
    registerNamedCommands();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the button bindings
    configureButtonBindings();

    // m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS. 
    //     new RunCommand(() -> {
    //         m_drivetrain.drive(
    //             (-MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband)),
    //             (-MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband)),
    //             (MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband)),
    //             true);
    //     }, m_drivetrain)
    // );

    // m_intake.setDefaultCommand(
      // new SpinIndexer(m_intake)  
    // );

    // SmartDashboard.putData("Auto Chooser", autoChooser); // Put the auto chooser on the dashboard
  } 


  private void registerNamedCommands() {
    // Register commands for auto
    // NamedCommands.registerCommand("shoot", new AimAndShootTurret(m_turret));
  }

    private void configureButtonBindings() {

    // m_turret.setDefaultCommand(new HoldZero(m_turret));

        // Intake in
        // primary.leftTrigger().toggleOnTrue(
        //     new IntakeIn(m_intake, -1)
        // );

        // Outtake
        // primary.leftBumper().whileTrue(
        //     new IntakeIn(m_intake, 1)
        // );

        // primary.a().whileTrue(
        //     new IntakeDown(m_intake)
        // );

        // primary.rightBumper().whileTrue(
        //     new PurgeTurret(m_turret)
        // );
        // primary.rightBumper().whileTrue(
        //     new SpinIndexer(m_spindexer)
        // );

        // primary.x().whileTrue(
        //     new HoodTarget(m_turret, 0)
        // );

        // Aim and shoot
        // primary.rightTrigger().whileTrue(
        //     new AimAndShootTurret(m_turret, m_drivetrain, primary, m_spindexer)
        // );
        // primary.rightTrigger().whileTrue(
        //     new ShakeIntake(m_intake)
        // );


        // primary.start().whileTrue(
        //     new DriveToPose(
        //         m_drivetrain,
        //         new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(0)),
        //         primary
        //     )
        // );
        primary.povRight().whileTrue(
          new MoveSnake(m_lights, 1)
        );
        primary.povLeft().whileTrue(
          new MoveSnake(m_lights, 2)
        );
        primary.povUp().whileTrue(
          new MoveSnake(m_lights, 3)
        );
        primary.povDown().whileTrue(
          new MoveSnake(m_lights, 1)
        );
    }

    // public Command getAutonomousCommand() {
    //     // return autoChooser.getSelected();
    // }
}
