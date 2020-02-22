package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeSuckCommand extends InstantCommand {
    public IntakeSuckCommand() {
        addRequirements(Robot.intakeSubsystem);
    }

    @Override 
    public void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.suck();
    }
    
}
