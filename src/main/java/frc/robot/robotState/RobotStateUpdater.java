package frc.robot.robotState;

import frc.robot.Robot;

public interface RobotStateUpdater {

    public void updateRobotState();
    default void enableUpdating(){
        Robot.addRobotStateUpdater(this);
    }
}