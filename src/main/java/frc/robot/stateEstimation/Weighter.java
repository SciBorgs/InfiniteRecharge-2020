package frc.robot.stateEstimation;

import frc.robot.RobotState;

import java.util.ArrayList;

public interface Weighter {
    RobotState weight(RobotState guess, ArrayList<RobotState> pastStates);
};