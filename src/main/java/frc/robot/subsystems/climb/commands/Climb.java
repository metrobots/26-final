// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
    private final ClimbSubsystem climbSubsystem;

    /** Creates a new ExampleCommand. */
    public Climb(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climbSubsystem.moveTowardsHeight(ClimbSubsystem.MOTOR_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climbSubsystem.moveTowardsHeight(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Climb is greater than 30 inches. This will need to be adjusted in the future.
        return climbSubsystem.getPositionInInches() > 30;
    }
}
