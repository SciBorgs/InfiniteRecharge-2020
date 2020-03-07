package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeStopCommand extends InstantCommand {
    public IntakeStopCommand() {
        requires(Robot.intakeSubsystem);
    }

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.stop();
    }
}
