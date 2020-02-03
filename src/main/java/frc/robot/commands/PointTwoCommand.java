package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PointTwoCommand extends InstantCommand {
    @Override
    public void execute() {
        Robot.CURRENT_DESTINATION = Robot.TEST_POINT_2.point;
        Robot.CURRENT_DESTINATION_HEADING = Robot.TEST_POINT_2.heading;
    }
}