package frc.robot.stateEstimation;

import frc.robot.RobotState;

import java.util.ArrayList;

public interface Updater {
    RobotState updateState(ArrayList<RobotState> pastRobotStates);
};