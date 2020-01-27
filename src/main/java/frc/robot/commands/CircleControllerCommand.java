package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.helpers.Geo;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends InstantCommand {

    private final String FILENAME = "CircleControllerCommand.java";
    CircleController circleController = new CircleController();

    @Override
    protected void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        circleController.update(Robot.getPos(), Robot.getHeading(), Robot.TEST_POINT_1.point, Robot.TEST_POINT_1.heading);
    }
}