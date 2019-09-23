package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotState.SD;

public interface Model{
    // Updates the global state in robotState
    public RobotState updatedRobotState();
    public Iterable<SD> getSDs();
}