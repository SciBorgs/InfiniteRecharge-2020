package frc.robot.commands.drive;

import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.helpers.DelayedPrinter;

public class TankDriveCommand extends InstantCommand {
    @Override 
    public void execute() {
        Robot.driveSubsystem.manualDriveMode();
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.driveSubsystem.setSpeed(Robot.oi.leftStick, Robot.oi.rightStick);
    }   
}
