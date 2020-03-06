package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotState.SD;

public class ElevatorControlCommand extends Command {

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        if (Robot.getState().getMapped(RobotState.BOOLEAN_MAPPING, SD.BallInElevator)) {
            Robot.hopperSubsystem.elevator();
        } else if (!Robot.shooterSubsystem.isRunning()) {
            Robot.hopperSubsystem.stopElevator();
        }
    }
}