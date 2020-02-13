package frc.robot.stateEstimation.interfaces;

import frc.robot.robotState.RobotStateHistory;;

public interface IllegalStateDeterminer {
    public boolean isStateIllegal(RobotStateHistory states);
}