package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.Lights;

public class MoveSnake extends Command {

    private final Lights lights;
    private final int direction;

    public MoveSnake(Lights lights, int direction) {
        this.lights = lights;
        this.direction = direction;

        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.steerSnake(direction);;
    }

    @Override
    public boolean isFinished() {
        return false; // stays active until interrupted
    }
}
