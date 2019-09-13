package frc.robot.stateEstimation;

import frc.robot.RobotState;

import java.util.ArrayList;

public interface Weighter {
    double weight(RobotState guess, ArrayList<RobotState> pastStates);
};