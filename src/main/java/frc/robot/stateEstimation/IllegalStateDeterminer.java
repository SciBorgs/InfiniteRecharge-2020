package frc.robot.stateEstimation;

import frc.robot.RobotStates;

public interface IllegalStateDeterminer {
    public boolean isIllegalState(RobotStates states);
}