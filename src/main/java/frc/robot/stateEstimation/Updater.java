package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotStates;
import frc.robot.RobotState.RS;

import java.util.Hashtable;

public interface Updater{
    RobotState updateState(RobotStates pastRobotStates);
    Hashtable<RS, Double> getStdDevs(); // Generally just an estimate
};