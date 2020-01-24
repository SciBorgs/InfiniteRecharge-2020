package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeReleaseCommand extends InstantCommand {

    private final String FILENAME = "IntakeReleaseCommand.java";

    public IntakeReleaseCommand() {
        requires(Robot.intakeSubsystem);
    }

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        Robot.intakeSubsystem.stop();
        System.out.println("test test test test fuck");
    }
}
