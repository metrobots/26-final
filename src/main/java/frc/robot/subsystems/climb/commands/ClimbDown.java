package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbDown extends Command {
    private final Climb climb;

    /**
     * Move climb mechanism to minimum extension.
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