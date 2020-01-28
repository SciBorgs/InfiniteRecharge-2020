package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends InstantCommand {

    private final String FILENAME = "CircleControllerCommand.java";
    CircleController circleController = new CircleController();

    @Override
    protected void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        circleController.update(Robot.CURRENT_DESTINATION, Robot.ORIGINAL_ANGLE);
        System.out.println("currDestination: " + Robot.CURRENT_DESTINATION);
    }
}