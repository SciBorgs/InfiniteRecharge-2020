package frc.robot.stateEstimation;

import frc.robot.RobotState;
import frc.robot.RobotStates;

import java.util.ArrayList;

public interface Weighter {
    double weight(RobotStates states);
};