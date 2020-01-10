package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ShooterCommand extends Command {

    @Override 
    protected void execute() {
        Robot.shooterSubsystem.setAngle(90);
        Robot.shooterSubsystem.setSpeedBottom(Robot.oi.leftStick.getY());
        Robot.shooterSubsystem.setSpeedTop(Robot.oi.rightStick.getY());
    }

	@Override
	protected boolean isFinished() {
		return false;
	}
}
