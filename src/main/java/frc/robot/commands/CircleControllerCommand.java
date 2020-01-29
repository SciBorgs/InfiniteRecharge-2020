package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.controllers.CircleController;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.logging.Logger.CommandStatus;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;

public class CircleControllerCommand extends InstantCommand {

    private final String FILENAME = "CircleControllerCommand.java";
    CircleController circleController = new CircleController();

    @Override
    protected void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        circleController.update(new Point(Utils.inchesToMeters(60), Utils.inchesToMeters(36)),Geo.HORIZONTAL_ANGLE);
        System.out.println("currDestination: " + Robot.CURRENT_DESTINATION);
        System.out.println("currHeading" + Robot.CURRENT_DESTINATION_HEADING);
    }
}