package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DecrementShootSpeedCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.shooterSubsystem.decrementShootSpeed();
    }
}
