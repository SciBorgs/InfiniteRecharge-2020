package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ShooterCommand extends InstantCommand {

    @Override 
    protected void execute() {
        Robot.shooterSubsystem.setSpeedBottom(Robot.oi.leftStick.getY());
        Robot.shooterSubsystem.setSpeedTop(Robot.oi.rightStick.getY());
    }
}