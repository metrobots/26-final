package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.turret.PurgeTurret;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeIn;
import frc.robot.subsystems.intake.commands.SpinIndexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.AimAndShootTurret;
import frc.robot.subsystems.turret.commands.HoodTarget;
import frc.robot.subsystems.turret.commands.ManualHood;
import frc.robot.subsystems.turret.commands.RotateTurret;
import frc.robot.subsystems.turret.commands.ShootTurret;
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

  // Command declarations — AimAndShootTurret is held as a field so SpinIndexer
  // can reference its isSustainedReady() signal.
  private final AimAndShootTurret m_aimAndShoot;
  private final SpinIndexer m_spinIndexer;

  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

  public RobotContainer() {

    m_turret = new Turret();
    m_drivetrain = new Drivetrain(m_turret);
    m_dashboard = new Dashboard();
    m_intake = new Intake();

    // Instantiate commands after subsystems
    m_aimAndShoot = new AimAndShootTurret(m_turret, m_drivetrain);
    m_spinIndexer = new SpinIndexer(m_intake);

    registerNamedCommands();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(
        new RunCommand(() -> {
            m_drivetrain.drive(
                (-MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband)),
                (-MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband)),
                (MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband)),
                true);
        }, m_drivetrain)
    );

    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    // Register commands for auto
    NamedCommands.registerCommand("shoot", new AimAndShootTurret(m_turret, m_drivetrain));
    NamedCommands.registerCommand("intake", new IntakeIn(m_intake, -0.4));
  }

  private void configureButtonBindings() {

    m_turret.setDefaultCommand(
        m_turret.run(() -> {
            if (primary.povLeft().getAsBoolean()) {
                m_turret.manualTurret(0.2);
            } else if (primary.povRight().getAsBoolean()) {
                m_turret.manualTurret(-0.2);
            } else {
                m_turret.manualTurret(0);
            }

            if (primary.povUp().getAsBoolean()) {
                m_turret.manualHood(-0.06);
            } else if (primary.povDown().getAsBoolean()) {
                m_turret.manualHood(0.06);
            } else {
                m_turret.manualHood(0);
            }
        })
    );

    // Intake in
    primary.leftTrigger().toggleOnTrue(
        new IntakeIn(m_intake, -0.7)
    );

    // Outtake
    primary.leftBumper().whileTrue(
        new IntakeIn(m_intake, 1)
    );

    primary.rightBumper().whileTrue(
      new PurgeTurret(m_turret)
    );

    // primary.a().whileTrue(
    //   new RotateTurret(m_turret, m_drivetrain)
    // );

    // primary.x().whileTrue(
    //   new SpinIndexer(m_intake)
    // );
    primary.start().whileTrue(new SpinIndexer(m_intake));
    // primary.back().whileTrue(new RotateTurret(m_turret, m_drivetrain));

    primary.x().whileTrue(
        new HoodTarget(m_turret, 20)
    );

    // Aim and shoot — right trigger holds both the turret command and the indexer
    primary.rightTrigger().whileTrue(m_aimAndShoot);
    primary.rightTrigger().whileTrue(m_spinIndexer);
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