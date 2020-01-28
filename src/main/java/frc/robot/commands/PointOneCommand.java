package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PointOneCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.CURRENT_DESTINATION = Robot.newDestPoint;
        Robot.CURRENT_DESTINATION_HEADING = Robot.newDestHeading;
    }
}