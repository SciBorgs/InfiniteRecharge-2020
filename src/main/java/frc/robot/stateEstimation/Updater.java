package frc.robot.stateEstimation;

import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;

import java.util.Hashtable;

public interface Updater{
    RobotState updateState(RobotStateHistory pastRobotStates);
    Hashtable<SD, Double> getStdDevs(); // Generally just an estimate
};