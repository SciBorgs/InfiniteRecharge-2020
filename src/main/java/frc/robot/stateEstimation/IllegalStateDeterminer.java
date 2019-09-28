package frc.robot.stateEstimation;

import frc.robot.RobotStateHistory;;

public interface IllegalStateDeterminer {
    public boolean isStateIllegal(RobotStateHistory states);
}