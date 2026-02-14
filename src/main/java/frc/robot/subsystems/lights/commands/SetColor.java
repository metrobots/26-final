package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.Lights;

public class SetColor extends Command {

    private final Lights lights;
    private final Color color;

    public SetColor(Lights lights, Color color) {
        this.lights = lights;
        this.color = color;

        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.setSolid(color);
    }

    @Override
    public boolean isFinished() {
        return false; // stays active until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing — next command decides LEDs
    }
}
