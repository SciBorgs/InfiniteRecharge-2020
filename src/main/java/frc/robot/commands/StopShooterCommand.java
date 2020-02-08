package frc.robot.commands;

import frc.robot.Robot;

import frc.robot.logging.Logger.CommandStatus;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopShooterCommand extends InstantCommand {   
    @Override
    protected void execute() {
        Robot.shooterSubsystem.testHoodSpark(0);
        Robot.shooterSubsystem.setShooterSpark(0);
    }
}
