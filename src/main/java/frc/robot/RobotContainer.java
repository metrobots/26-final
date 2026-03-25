package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeDown;
import frc.robot.subsystems.intake.commands.IntakeIn;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.commands.SpinIndexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.AimAndShootTurret;
import frc.robot.subsystems.turret.commands.HoodTarget;
import frc.robot.subsystems.turret.commands.PurgeTurret;
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
  private final SendableChooser<Command> autoChooser;

  // Subsystem declarations
  final Drivetrain m_drivetrain;
  final Turret m_turret;
  final Intake m_intake;
  final Dashboard m_dashboard;
  final Spindexer m_spindexer;

  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

  public RobotContainer() {

    m_turret = new Turret();
    m_drivetrain = new Drivetrain(m_turret);
    m_dashboard = new Dashboard();
    m_intake = new Intake();
    m_spindexer = new Spindexer();

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

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

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("shoot", new AimAndShootTurret(m_turret, m_drivetrain, primary, m_spindexer).withTimeout(8.0));
    NamedCommands.registerCommand("intake", new IntakeIn(m_intake, -0.7));
    NamedCommands.registerCommand("down", new IntakeDown(m_intake).withTimeout(3));
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
        new IntakeIn(m_intake, -0.9)
    );

    // Outtake
    primary.leftBumper().whileTrue(
        new IntakeIn(m_intake, 1)
    );

    primary.a().whileTrue(
        new IntakeDown(m_intake)
    );

    primary.rightBumper().whileTrue(
        new PurgeTurret(m_turret)
    );

    // Manual indexer override — kept for use independent of shooting
    primary.start().whileTrue(new SpinIndexer(m_spindexer));

    primary.x().whileTrue(
        new HoodTarget(m_turret, 0)
    );

    // Aim and shoot — AimAndShootTurret now internally gates the indexer via readyToShoot
    primary.rightTrigger().whileTrue(
        new AimAndShootTurret(m_turret, m_drivetrain, primary, m_spindexer)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}