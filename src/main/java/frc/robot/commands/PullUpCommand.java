package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
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
        return Utils.inRange(ENDING_GOAL, Robot.get(SD.ClimberHeight), ERROR);
    }

    @Override
    protected void end() {
        Robot.climberSubsystem.setLiftSpeed(0);
    }
}