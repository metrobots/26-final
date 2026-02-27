package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbDown extends Command {
    private final Climb climb;

    /** 
     * Move the climb down at a given speed while the command is active. 
     * @param climbSubsystem the climb subsystem
     * @param speed negative value (-1-0) for downward movement
     */
    public ClimbDown(Climb climbSubsystem) {
        this.climb = climbSubsystem;
    }

    @Override
    public void initialize() {
        climb.setDesiredExtension(0);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return climb.getExtension() <= 0;
    }
}