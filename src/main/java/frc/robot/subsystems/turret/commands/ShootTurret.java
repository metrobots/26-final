package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// TUNING DIRECTIONS FOR SYSID CHARACTERIZATION:
/*
1. ADD TEMP BINDINGS: 
driverController.a().whileTrue(
    turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

driverController.b().whileTrue(
    turret.sysIdDynamic(SysIdRoutine.Direction.kForward));

driverController.x().whileTrue(
    turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

driverController.y().whileTrue(
    turret.sysIdDynamic(SysIdRoutine.Direction.kReverse)); 
    
2. Open Driver Station, open Glass Log Viewer and connect to robot. Glass will automatically detect the SysId data stream and start logging when you run the tests.

3. Run Tests (THIS ORDER MATTERS) You will run four tests.

    3a. Test 1 — Quasistatic Forward
        Enable robot
        Hold A
        Let it slowly ramp to near max speed
        Release before it saturates at 12V
        Duration: ~5–7 seconds
        Disable robot.

    3b. Test 2 — Dynamic Forward
        Enable robot
        Hold B
        It will jump to high voltage instantly
        Let it reach steady speed
        Release
        Duration: ~3–4 seconds
        Disable robot.

    3c. Test 3 — Quasistatic Reverse
        Same as forward, but hold X

    3d. Test 4 — Dynamic Reverse
        Same as forward, but hold Y

4. Stop recording in Driver Station, Download the .wpilog file.

5. Open WPILib SysId tool.
    Drag in the .wpilog
    Select mechanism: flywheel
    Select:
        Voltage
        Angular velocity (RPS)
    Click Analyze
*/

import frc.robot.subsystems.turret.Turret;

public class ShootTurret extends Command {

    private final Turret turret;

    // 2000 RPM ≈ 33.3 RPS
    private static final double TARGET_RPS = -33.3;

    // Replace with SysId values after characterization
    private final PIDController pid = new PIDController(0.0005, 0.0, 0.0);
    private final SimpleMotorFeedforward ff =
            new SimpleMotorFeedforward(0.2, 0.12);

    public ShootTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
        pid.setTolerance(1.0); // 1 RPS tolerance
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        double currentRPS = turret.getFlywheelVelocity();

        SmartDashboard.putNumber("Flywheel RPS", currentRPS);
        SmartDashboard.putNumber("Target RPS", TARGET_RPS);

        double ffVolts = ff.calculate(TARGET_RPS);
        double pidVolts = pid.calculate(currentRPS, TARGET_RPS);

        double totalVolts = ffVolts + pidVolts;

        turret.setFlywheelVoltage(totalVolts);

        if (pid.atSetpoint()) {
            turret.spinFeed(0.5);
        } else {
            turret.spinFeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setFlywheelVoltage(0);
        turret.spinFeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}