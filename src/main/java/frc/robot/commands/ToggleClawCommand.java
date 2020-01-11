package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleClawCommand extends InstantCommand {
    private final String FILENAME = "ToggleClawCommand.java";

    public ToggleClawCommand() {}
    
    @Override
    protected void execute() {
        Robot.climberSubsystem.toggleClaw();
    }
}