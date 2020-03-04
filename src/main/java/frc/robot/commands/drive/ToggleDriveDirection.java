package frc.robot.commands.drive;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleDriveDirection extends InstantCommand {

    @Override 
    protected void execute() {
        Robot.driveSubsystem.toggleDriveDirection();
    }
}
