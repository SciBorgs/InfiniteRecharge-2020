package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotState.RS;

public interface Model{
    // Updates the global state in robotState
    public RobotState updatedRobotState();
    public Iterable<RS> getRSs();
}