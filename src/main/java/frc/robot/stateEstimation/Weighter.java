package frc.robot.stateEstimation;

import frc.robot.RobotStates;

public interface Weighter {
    double weight(RobotStates states);
};