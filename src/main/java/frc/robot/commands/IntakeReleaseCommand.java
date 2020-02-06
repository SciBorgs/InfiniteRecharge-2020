package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeReleaseCommand extends InstantCommand {

    private final String FILENAME = "IntakeReleaseCommand.java";

    public IntakeReleaseCommand() {
        addRequirements(Robot.intakeSubsystem);
    }

    @Override 
    public void execute() {
	    Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        Robot.intakeSubsystem.stop();
    }
}
