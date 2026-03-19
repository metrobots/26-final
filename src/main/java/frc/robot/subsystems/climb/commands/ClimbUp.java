package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.utils.Constants.ClimbConstants;

public class ClimbUp extends Command {
    private final Climb climb;

    /**
     * Move climb mechanism to maximum extension.
     */
    public ClimbUp(Climb climbSubsystem) {
        this.climb = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climb.setDesiredExtension(ClimbConstants.maxExtensionInInches);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return climb.getExtension() >= ClimbConstants.maxExtensionInInches;
    }
}
