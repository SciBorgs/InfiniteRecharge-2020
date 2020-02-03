package frc.robot.commands;

import frc.robot.Robot;

import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class TorqueDriveCommand extends InstantCommand {
    @Override protected void execute() {
        Robot.logger.logCommandStatus(CommandStatus.Executing);
        // One controller controls turning percent, one controls velocity
        double forward = Robot.oi.leftStick.getProcessedY();
        double turnAmount = -Robot.oi.rightStick.getProcessedX();
       // double turnAmount = -Robot.oi.leftStick.getX();
        Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(forward * 2, turnAmount);

        
    }
}