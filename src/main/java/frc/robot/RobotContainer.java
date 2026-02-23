package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeIn;
import frc.robot.subsystems.intake.commands.IntakePivot;
import frc.robot.subsystems.intake.commands.ManualPivotIntake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.subsystems.turret.commands.TestTurret;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;

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
  final Dashboard m_dashboard;
  
  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

   public RobotContainer() {

    m_drivetrain = new Drivetrain();
    m_turret = new Turret();
    m_intake = new Intake();
    m_dashboard = new Dashboard();
    
    
    registerNamedCommands();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS. 
        new RunCommand(() -> {
            m_drivetrain.drive(
                (-MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband)),
                (-MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband)),
                (MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband)),
                true);
        }, m_drivetrain)
    );

    // if you do auto choose add it here
  } 


  private void registerNamedCommands() {
    // Register commands for auto
  }

  private void configureButtonBindings() {
    //intake pivot command
    primary.a().whileTrue(
      new ManualPivotIntake(m_intake)
    );

    // intake command
    primary.leftTrigger().whileTrue(
      new IntakeIn(m_intake, -1)
    );

    // outtake command
    primary.leftBumper().whileTrue(
      new IntakeIn(m_intake, 1)
    );

    // turret commands
    m_turret.setDefaultCommand(
        m_turret.run(() -> {
            // ===== Turret rotation =====
            if (primary.povLeft().getAsBoolean()) {
                m_turret.manualTurret(0.08);
            } 
            else if (primary.povRight().getAsBoolean()) {
                m_turret.manualTurret(-0.08);
            } 
            else {
                m_turret.manualTurret(0);
            }

            // ===== Hood control =====
            if (primary.povUp().getAsBoolean()) {
                m_turret.manualHood(-0.06);
            } 
            else if (primary.povDown().getAsBoolean()) {
                m_turret.manualHood(0.06);
            } 
            else {
                m_turret.manualHood(0);
            }

        })
    );

    // auto align turret command
    primary.rightTrigger().whileTrue(
      new TestTurret(m_turret)
    );

    primary.y().whileTrue(
      new IntakePivot(m_intake)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
