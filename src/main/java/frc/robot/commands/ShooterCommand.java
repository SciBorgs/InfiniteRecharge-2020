package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ShooterCommand extends InstantCommand {
    public ShooterCommand() {
        requires(Robot.shooterSubsystem);
    }

    @Override 
    protected void execute() {
        Robot.shooterSubsystem.logSpeed();
    }
}