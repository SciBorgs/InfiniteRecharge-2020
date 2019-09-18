package frc.robot.stateEstimation;

import frc.robot.RobotState.RS;

public interface Model{
    public void updateRobotState();
    public Iterable<RS> getRSs();
}