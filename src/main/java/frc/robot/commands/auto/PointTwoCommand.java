package frc.robot.commands.auto;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PointTwoCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.CURRENT_DESTINATION = Robot.TEST_POINT_2.point;
        Robot.CURRENT_DESTINATION_HEADING = Robot.TEST_POINT_2.heading;
    }
}