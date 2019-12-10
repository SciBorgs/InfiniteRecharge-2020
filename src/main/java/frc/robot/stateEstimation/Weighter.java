package frc.robot.stateEstimation;

import frc.robot.robotState.RobotStateHistory;

public interface Weighter {
    double weight(RobotStateHistory states);
};