package frc.robot.stateEstimation;

import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotState.SD;

public interface Model{
    // Updates the global state in robotState
    public RobotState updatedRobotState();
    public Iterable<SD> getSDs();
}