package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logging.Logger.CommandStatus;
import frc.robot.helpers.DelayedPrinter;

public class CircleControllerCommand extends InstantCommand {
    CircleController circleController = new CircleController();

    @Override
    public void execute() {
        Robot.logger.logCommandStatus(CommandStatus.Executing);
        circleController.update(Robot.CURRENT_DESTINATION, Robot.CURRENT_DESTINATION_HEADING);
    }
}