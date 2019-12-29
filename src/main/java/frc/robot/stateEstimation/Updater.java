package frc.robot.stateEstimation;

import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;

import java.util.Hashtable;

public interface Updater{
    // Update updates/changes the current state! It does not say what the next state should be
    // Theoeretically, calling updateState on a given RobotStateHistory twice in quick succession should be
    //   equivilant to calling it once
    void updateState(RobotStateHistory pastRobotStates);
    Hashtable<SD, Double> getStdDevs(); // Generally just an estimate
};