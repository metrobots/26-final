package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveInDirection extends Command {
    public enum DriveDirection {
        Forward,
        Left,
        Backward,
        Right,
    }

    private final double speed = 0.2;

    private final Drivetrain drivetrain;
    private final DriveDirection direction;

    public DriveInDirection(Drivetrain drivetrain, DriveDirection dir) {
        this.drivetrain = drivetrain;
        this.direction = dir;
    }

    @Override
    public void execute() {
        switch (direction) {
            case Forward:
                this.drivetrain.drive(speed, 0, 0, false);
                break;
            case Backward:
                this.drivetrain.drive(-speed, 0, 0, false);
                break;
            case Left:
                this.drivetrain.drive(0, speed, 0, false);;
                break;
            case Right:
                this.drivetrain.drive(0, -speed, 0, false);
                break;
            default:
                break;
        }
    }
}
