package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PointTwoCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.CURRENT_DESTINATION = Robot.TEST_POINT_2.point;
    }
}