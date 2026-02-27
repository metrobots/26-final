package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbUp extends Command {
    private final Climb climb;

    /** 
     * Move the climb up at a given speed while the command is active. 
     * @param climbSubsystem the climb subsystem
     * @param speed positive value (0-1) for upward movement
     */
    public ClimbUp(Climb climbSubsystem) {
        this.climb = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climb.setDesiredExtension(Climb.maxExtensionInInches);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return climb.getExtension() >= Climb.maxExtensionInInches;
    }
}