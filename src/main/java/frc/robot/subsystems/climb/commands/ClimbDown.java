package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbDown extends Command {
    private final Climb climb;
    private final double speed;

    /** 
     * Move the climb down at a given speed while the command is active. 
     * @param climbSubsystem the climb subsystem
     * @param speed negative value (-1-0) for downward movement
     */
    public ClimbDown(Climb climbSubsystem, double speed) {
        this.climb = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {

        climb.positionalMove(climb.getMinHeight(), speed);

    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the button is released
    }
}