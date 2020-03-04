package frc.robot.robotState;

import frc.robot.Robot;

public interface RobotStateUpdater {

    public void updateRobotState();
    default void automateStateUpdating(){
        Robot.addRobotStateUpdater(this);
    }
    default boolean ignore(){return false;}
}