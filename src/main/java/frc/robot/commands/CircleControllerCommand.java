package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.helpers.Geo;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.logging.Logger.CommandStatus;

public class CircleControllerCommand extends Command {

    private final String FILENAME = "CircleControllerCommand.java";
    private static final double DISTANCE_TOLERANCE = .1;
    CircleController circleController = new CircleController();

    public CircleControllerCommand(){
    }

    @Override
    protected void initialize() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Initializing);
    }

    @Override
    protected void execute() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Executing);
        circleController.update(Robot.getPos(), Robot.getHeading(), Robot.TEST_POINT_1.point, Robot.TEST_POINT_1.heading);
    }

    @Override
    protected boolean isFinished(){
        return Geo.getDistance(Robot.getPos(), Robot.TEST_POINT_1.point) < DISTANCE_TOLERANCE;
    }

    @Override
    protected void end() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Ending);
    }

    @Override
    protected void interrupted() {
        Robot.logger.logCommandStatus(FILENAME, CommandStatus.Interrupted);
        end();
    }
}