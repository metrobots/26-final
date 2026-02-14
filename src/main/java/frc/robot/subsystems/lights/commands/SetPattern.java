package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.Lights;

public class SetPattern extends Command {

    private final Lights lights;
    private final Lights.Pattern pattern;

    public SetPattern(Lights lights, Lights.Pattern pattern) {
        this.lights = lights;
        this.pattern = pattern;

        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.setPattern(pattern);
    }

    @Override
    public boolean isFinished() {
        return false; // animation runs continuously
    }

    @Override
    public void end(boolean interrupted) {
        // optional: lights.off();
    }
}
