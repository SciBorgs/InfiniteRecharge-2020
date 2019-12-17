package frc.robot.stateEstimation;

import java.util.Map;

import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;

public class GaussianWeighter implements Weighter {

    // Allows an Updater to essentially become a Weighter (or Predictor)
    private Updater updater;

    public GaussianWeighter(Updater updater){
        this.updater = updater;
    }

    private double computeGaussian(double mean, double x, double variance) {
        return 1 / (variance * Math.sqrt(2 * Math.PI)) * Math.pow(Math.E, -(1 / 2) * Math.pow(((x - mean) / variance), 2));
    }
    
    public double weight(RobotStateHistory stateHistory){
        RobotState state = stateHistory.currentState();
        RobotStateHistory pastStates = stateHistory.copy();
        pastStates.dropFirstState();
        RobotState expectedState = updater.updateState(pastStates);
        double weight = 1;
        for(Map.Entry<SD, Double> sdVariance : updater.getStdDevs().entrySet()){
            weight *= computeGaussian(expectedState.get(sdVariance.getKey()), 
                                              state.get(sdVariance.getKey()),
                                                        sdVariance.getValue());
        }
        return weight;
    }
}