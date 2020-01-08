package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Command;

public class PullUpCommand extends Command {
    private final double ERROR = 0.1;
    private final double PULL_UP_AMOUNT = 0.3;
    private final double ENDING_GOAL;
    
    public PullUpCommand() {
        ENDING_GOAL = Robot.get(SD.ClimberHeight) - PULL_UP_AMOUNT;
    }
    
    @Override
    protected void execute() {
        Robot.climberController.moveToHeight(ENDING_GOAL);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(ENDING_GOAL - Robot.get(SD.ClimberHeight)) < ERROR;
    }
}