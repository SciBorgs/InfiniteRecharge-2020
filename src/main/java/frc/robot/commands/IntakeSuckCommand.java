package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeSuckCommand extends InstantCommand {
    public IntakeSuckCommand() {
        requires(Robot.intakeSubsystem);
    }

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.suck();
    }
    
}
