package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ManualShootCommand extends InstantCommand {
    public ManualShootCommand() {
        requires(Robot.shooterSubsystem);
    }
    
    @Override
    protected void execute() {
        Robot.shooterSubsystem.shoot(Robot.oi.rightStick.getY());
    }
}