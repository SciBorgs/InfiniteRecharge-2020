package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ToggleGearsCommand extends InstantCommand {
    private final String FILENAME = "ToggleGearsCommand.java";

    @Override
    protected void execute() {
        Robot.gearBoxSubsystem.toggleGears();
    }
}