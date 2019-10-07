package frc.robot.stateEstimation;

import frc.robot.RobotStateHistory;

public interface Weighter {
    double weight(RobotStateHistory states);
};