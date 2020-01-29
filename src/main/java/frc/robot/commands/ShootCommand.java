package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ShootCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.shooterSubsystem.shoot(Robot.oi.rightStick.getY());
    }
}