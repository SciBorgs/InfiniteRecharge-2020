package frc.robot.stateEstimation.sciModels;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.stateEstimation.interfaces.Model;

public class JerkModel implements Model {

    public SciSpark spark;

    public JerkModel(SciSpark spark) {
        this.spark = spark;
    }

    @Override
    public void updateRobotState() {
        Robot.optionalSet(this.spark.jerkSD, StateInfo.getRateOfChange(this.spark.accelSD.get(), spark.getMovementPrecision()));
    }

    @Override
    public Iterable<SD> getSDs() {
        ArrayList<SD> sd = new ArrayList<SD>();
        sd.add(this.spark.jerkSD.get());
        return sd;
    }

}