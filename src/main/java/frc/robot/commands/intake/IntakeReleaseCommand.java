package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeReleaseCommand extends InstantCommand {
    public IntakeReleaseCommand() {
        addRequirements(Robot.intakeSubsystem);
    }

    @Override 
    public void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.stop();
    }
}
