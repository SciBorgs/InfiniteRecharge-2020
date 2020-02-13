package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.helpers.DelayedPrinter;

public class TankDriveCommand extends InstantCommand {
    @Override 
    protected void execute() {
        Robot.driveSubsystem.manualDriveMode();
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.driveSubsystem.setSpeed(Robot.oi.leftStick, Robot.oi.rightStick);
    }   
}
