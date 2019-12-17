package frc.robot.stateEstimation;

import frc.robot.robotState.RobotStateHistory;;

public interface IllegalStateDeterminer {
    public boolean isStateIllegal(RobotStateHistory states);
}