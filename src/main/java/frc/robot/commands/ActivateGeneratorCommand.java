package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ActivateGeneratorCommand extends CommandGroup {
    private final String FILENAME = "ActivateGeneratorCommand.java";

    public ActivateGeneratorCommand() {
        requires(Robot.climberSubsystem)
        addSequential(new PullUpCommand);
        addSequential(new ReachForBarCommand);
    }
}