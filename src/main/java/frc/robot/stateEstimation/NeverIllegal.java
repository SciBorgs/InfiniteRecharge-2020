package frc.robot.stateEstimation;

import frc.robot.RobotStates;

public class NeverIllegal implements IllegalStateDeterminer{
    @Override
    public boolean isIllegalState(RobotStates state){
        return false;
    }
}