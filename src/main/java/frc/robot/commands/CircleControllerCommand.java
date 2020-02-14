package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.shapes.Waypoint;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends InstantCommand {

    private CircleController circleController = new CircleController();
    private Waypoint waypoint;

    public CircleControllerCommand ()                  { waypoint = Robot.CURRENT_DESTINATION; }
    public CircleControllerCommand (Waypoint waypoint) { this.waypoint = waypoint; }

    @Override
    protected void execute() {
        circleController.update(waypoint);
        DelayedPrinter.print("currDest: " + Robot.CURRENT_DESTINATION, 20);
        Robot.logger.logCommandStatus(CommandStatus.Executing);
    }
}