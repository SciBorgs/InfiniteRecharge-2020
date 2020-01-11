package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ToggleOutputCommand extends InstantCommand {
    private final String FILENAME = "ToggleOutputCommand.java";

    @Override
    protected void execute() {
        Robot.gearBoxSubsystem.toggleOutput();
    }
}