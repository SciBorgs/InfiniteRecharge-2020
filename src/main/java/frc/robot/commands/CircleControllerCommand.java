package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logging.Logger.CommandStatus;
import frc.robot.helpers.DelayedPrinter;

public class CircleControllerCommand extends InstantCommand {

    private final String FILENAME = "CircleControllerCommand.java";
    CircleController circleController = new CircleController();

    @Override
    public void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        circleController.update(Robot.CURRENT_DESTINATION, Robot.CURRENT_DESTINATION_HEADING);
    }
}