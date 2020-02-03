package frc.robot.stateEstimation.sciModels;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.stateEstimation.interfaces.Model;

public class SnapModel implements Model {

    public SciSpark spark;

    public SnapModel(SciSpark spark) {
        this.spark = spark;
    }

    @Override
    public void updateRobotState() {
        Robot.optionalSet(this.spark.snapSD, StateInfo.getRateOfChange(this.spark.snapSD.get(), spark.getMovementPrecision()));
    }

    @Override
    public Iterable<SD> getSDs() {
        ArrayList<SD> sd = new ArrayList<SD>();
        sd.add(this.spark.snapSD.get());
        return sd;
    }

}