package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.shapes.Waypoint;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends Command {

    public CircleController circleController;
    private Waypoint waypoint;
    private int counter = 0;
    private int counterReq = 5;
    private double prevDist;

    public CircleControllerCommand (Waypoint waypoint) { circleController = new CircleController(); }
    
    public CircleControllerCommand (Waypoint waypoint, double speed) {
        this.waypoint = waypoint;
        circleController = new CircleController(speed);
        this.prevDist = Geo.getDistance(waypoint.point, Robot.getPos());
    }

    @Override
    protected void execute() {
        circleController.update(waypoint);
        DelayedPrinter.print("currDest: " + waypoint);
        Robot.logger.logCommandStatus(CommandStatus.Executing);
        double dist = Geo.getDistance(waypoint.point, Robot.getPos());
        if (dist > this.prevDist) {
            counter++;
        } else {
            counter = 0;
        }
        this.prevDist = dist;
    }

    @Override
    protected boolean isFinished() {
        return circleController.isFinished() || this.counter > this.counterReq;
    }

    @Override
    protected void end() {
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        System.out.println("CCCommand finished: " + waypoint);
        Robot.driveSubsystem.setTank(0, 0);
    }
}