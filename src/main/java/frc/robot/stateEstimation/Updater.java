package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotStateHistory;
import frc.robot.RobotState.SD;

import java.util.Hashtable;

public interface Updater{
    RobotState updateState(RobotStateHistory pastRobotStates);
    Hashtable<SD, Double> getStdDevs(); // Generally just an estimate
};