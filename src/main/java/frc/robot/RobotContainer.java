package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeIn;
import frc.robot.subsystems.intake.commands.IntakePurge;
import frc.robot.subsystems.intake.commands.ManualPivotIntake;
import frc.robot.subsystems.intake.commands.SpinIndexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.turret.commands.AimTurret;
import frc.robot.subsystems.turret.commands.ManualHood;
import frc.robot.subsystems.turret.commands.ManualTurret;
import frc.robot.subsystems.turret.commands.PurgeShooter;
import frc.robot.subsystems.turret.commands.ShootTurret;
import frc.robot.subsystems.turret.commands.TestShooter;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.utilcommands.Turtle;

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
  final Drivetrain m_drivetrain;
  final Turret m_turret;
  final Intake m_intake;
  private final TurretHoodTable hoodTable = new TurretHoodTable();
  
  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

   public RobotContainer() {

    m_drivetrain = new Drivetrain();
    m_turret = new Turret();
    m_intake = new Intake();
    
    
    registerNamedCommands();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS. 
        new RunCommand(() -> {
            m_drivetrain.drive(
                (-MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband)) * 0.5,
                (-MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband)) * 0.5,
                (MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband)),
                true);
        }, m_drivetrain)
    );

    m_intake.setDefaultCommand(
      new SpinIndexer(m_intake, 0.2)  
    );

    // SmartDashboard.putData("Auto Chooser", autoChooser); // Put the auto chooser on the dashboard
  } 


  private void registerNamedCommands() {
    // Register commands for auto
  }

  private void configureButtonBindings() {
    // Configure bindings for controller
    //  m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS
    //     new RunCommand(() -> {
    //         m_drivetrain.drive(
    //             MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband),
    //             MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband),
    //             true);
    //     }, m_drivetrain)
    // );

    // primary.a().whileTrue(
    //   new ManualPivotIntake(m_intake, -0.15)
    // );


    // test turret commands
    primary.povLeft().whileTrue(new TurnTurret(0.1, m_turret));
    primary.povRight().whileTrue(new TurnTurret(-0.1, m_turret));

    // y to align + range HUB
    // primary.y().toggleOnTrue(
    //   new AimTurret(m_turret, hoodTable)
    // );

    

    // // x to turtle (bring everything inside frame)
    // primary.x().toggleOnTrue(
    //   new Turtle(m_turret)
    // );

    // RT to shoot
    // primary.rightTrigger().toggleOnTrue(
    //   new ShootTurret(m_turret)
    // );
    // // RB to purge shooter
    // primary.rightBumper().toggleOnTrue(
    //   new PurgeShooter(m_turret)
    // );

    // // LT to intake
    // primary.leftTrigger().toggleOnTrue(
    //   new IntakeIn(m_intake, 1)
    // );
    // // LB to purge intake
    // primary.leftBumper().toggleOnTrue(
    //   new IntakePurge()
    // );

    // // d-pad to manual move turret + hood
    // primary.povUp().toggleOnTrue(
    //   new ManualHood(m_turret, 1)
    // );
    // primary.povDown().toggleOnTrue(
    //   new ManualHood(m_turret, -1)
    // );
    // primary.povLeft().toggleOnTrue(
    //   new ManualTurret(m_turret, -1)
    // );
    // primary.povRight().toggleOnTrue(
    //   new ManualTurret (m_turret, 1)
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
