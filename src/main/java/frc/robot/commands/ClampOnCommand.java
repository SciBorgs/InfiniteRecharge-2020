package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ClampOnCommand extends InstantCommand {
    private final String FILENAME = "ClampOnCommand.java";

    public ClampOnCommand() {}
    
    @Override
    protected void execute() {
        Robot.climberSubsystem.close();
    }
}