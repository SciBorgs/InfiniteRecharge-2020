package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj.command.Command;

public class ShooterCommand extends Command {

    @Override 
    protected void execute() {
        Robot.shooterSubsystem.setAngle(90);
        Robot.shooterSubsystem.setSpeed(1);
    }

	@Override
	protected boolean isFinished() {
		return false;
	}
}
