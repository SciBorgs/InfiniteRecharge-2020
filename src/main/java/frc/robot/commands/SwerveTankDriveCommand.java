package frc.robot.commands;

import frc.robot.Robot;

import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SwerveTankDriveCommand extends InstantCommand {
    private final String FILENAME = "SwerveTankDriveCommand.java";
   
    
    public SwerveTankDriveCommand(){}

    @Override protected void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);

        // One controller controls turning percent, one controls velocity
        double forward = Robot.oi.leftStick.getProcessedY();
        double turnAmount = Robot.oi.rightStick.getX();
        Robot.driveSubsystem.setSpeedTankForwardTurningPercentage(forward, turnAmount);

        
    }
}
