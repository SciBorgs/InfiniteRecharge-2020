package frc.robot.stateEstimation;

import frc.robot.robotState.RobotStateHistory;

public class NeverIllegal implements IllegalStateDeterminer{
    @Override
    public boolean isStateIllegal(RobotStateHistory state){
        return false;
    }
}