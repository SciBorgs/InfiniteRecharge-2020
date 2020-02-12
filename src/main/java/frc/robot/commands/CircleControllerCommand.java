package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.helpers.DelayedPrinter;

public class CircleControllerCommand extends InstantCommand {

    CircleController circleController = new CircleController();

    @Override
    protected void execute() {
        circleController.update(Robot.CURRENT_DESTINATION.point, Robot.CURRENT_DESTINATION.heading);
        DelayedPrinter.print("currDest: " + Robot.CURRENT_DESTINATION, 20);
    }
}