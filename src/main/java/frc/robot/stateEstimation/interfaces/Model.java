package frc.robot.stateEstimation.interfaces;

import frc.robot.robotState.RobotState.SD;

public interface Model{
    // Updates the global state in robotState
    public void updateRobotState();
    public Iterable<SD> getSDs();
}