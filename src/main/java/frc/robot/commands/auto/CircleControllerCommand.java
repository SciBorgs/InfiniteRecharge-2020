package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.shapes.Waypoint;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends Command {

    public CircleController circleController = new CircleController();
    private Waypoint waypoint;
    private static final double TOLERANCE = .2;

    public CircleControllerCommand ()                  { waypoint = Robot.CURRENT_DESTINATION; }
    public CircleControllerCommand (Waypoint waypoint) { this.waypoint = waypoint; }

    @Override
    protected void execute() {
        circleController.update(waypoint);
        DelayedPrinter.print("currDest: " + waypoint);
        Robot.logger.logCommandStatus(CommandStatus.Executing);
    }

    @Override
    protected boolean isFinished() {
        return circleController.isFinished();
    }
}