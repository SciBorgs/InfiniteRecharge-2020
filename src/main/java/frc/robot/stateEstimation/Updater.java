package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotState.RS;

import java.util.ArrayList;
import java.util.Hashtable;

public interface Updater{
    RobotState updateState(ArrayList<RobotState> pastRobotStates);

    Hashtable<RS, Double> getVariances();
};