package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ManualHoodCommand extends InstantCommand {
    public ManualHoodCommand() {
        requires(Robot.shooterSubsystem);
    }
    
    @Override
    protected void execute() {
        Robot.shooterSubsystem.setHoodAngle(Robot.oi.leftStick.getY() / 3.0);    
    }
}