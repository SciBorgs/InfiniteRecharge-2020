package frc.robot.stateEstimation;

import java.util.ArrayList;
import java.util.Map;

import frc.robot.RobotState;
import frc.robot.RobotStates;
import frc.robot.RobotState.RS;

public class GaussianWeighter implements Weighter {

    // Allows an Updater to essentially become a Weighter (or Predictor)
    private Updater updater;

    public GaussianWeighter(Updater updater){
        this.updater = updater;
    }

    private double computeGaussian(double mean, double x, double variance) {
        return 1 / (variance * Math.sqrt(2 * Math.PI)) * Math.pow(Math.E, -(1 / 2) * Math.pow(((x - mean) / variance), 2));
    }
    
    public double weight(RobotStates states){
        RobotState state = states.currentState();
        RobotStates pastStates = states.copy();
        pastStates.dropFirstState();
        RobotState expectedState = updater.updateState(pastStates);
        double weight = 1;
        for(Map.Entry<RS, Double> rsVariance : updater.getStdDevs().entrySet()){
            weight *= computeGaussian(expectedState.get(rsVariance.getKey()), 
                                              state.get(rsVariance.getKey()),
                                                        rsVariance.getValue());
        }
        return weight;
    }
}