package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeSuckCommand extends CommandBase {

    private final String FILENAME = "IntakeSuckCommand.java";

    public IntakeSuckCommand() {
        addRequirements(Robot.intakeSubsystem);
    }

    @Override 
    public void execute() {
	    Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        Robot.intakeSubsystem.suck();
    }
    
}
