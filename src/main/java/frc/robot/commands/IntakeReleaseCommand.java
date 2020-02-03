package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeReleaseCommand extends InstantCommand {
    public IntakeReleaseCommand() {
        requires(Robot.intakeSubsystem);
    }

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.stop();
    }
}
