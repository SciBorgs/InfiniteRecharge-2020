package frc.robot.stateEstimation.interfaces;

import frc.robot.robotState.RobotStateHistory;

public interface Weighter {
    double weight(RobotStateHistory states);
};