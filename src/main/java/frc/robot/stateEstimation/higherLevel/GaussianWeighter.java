package frc.robot.stateEstimation.higherLevel;

import java.util.Map;

import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;
import frc.robot.stateEstimation.interfaces.*;

public class GaussianWeighter implements Weighter {

    // Allows an Updater to essentially become a Weighter (or Predictor)
    private Updater updater;

    public GaussianWeighter(Updater updater){
        this.updater = updater;
    }

    // How unlikely the x would be, given a deviation
    private double computeGaussian(double mean, double x, double variance) {
        return 1 / (variance * Math.sqrt(2 * Math.PI)) * Math.pow(Math.E, -(1 / 2) * Math.pow(((x - mean) / variance), 2));
    }
    
    // Weights based on multiplied gaussians
    public double weight(RobotStateHistory stateHistory){
        RobotState state = stateHistory.currentState();
        RobotStateHistory statesCopy = stateHistory.copy();
        updater.updateState(statesCopy);
        RobotState expectedState = statesCopy.currentState();
        double weight = 1;
        for(Map.Entry<SD, Double> sdVariance : updater.getStdDevs().entrySet()){
            weight *= computeGaussian(expectedState.get(sdVariance.getKey()), 
                                              state.get(sdVariance.getKey()),
                                                        sdVariance.getValue());
        }
        return weight;
    }
}