package frc.robot.stateEstimation;

import java.util.Hashtable;
import java.util.Set;

import frc.robot.Robot;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;

public class MaybeDefaultUpdater implements Model, Updater {
    private MaybeUpdater maybeUpdater;
    private Updater defaultUpdater;

    public MaybeDefaultUpdater(MaybeUpdater maybeUpdater, Updater defaultUpdater){
        this.maybeUpdater = maybeUpdater;
        this.defaultUpdater = defaultUpdater;
    }

    public Updater getCurrentUpdater(){
        return this.maybeUpdater.canUpdate() ? this.maybeUpdater :  this.defaultUpdater;
    }

    @Override public Hashtable<SD, Double> getStdDevs(){
        return getCurrentUpdater().getStdDevs();
    }

    @Override
    public Set<SD> getSDs(){
        return getStdDevs().keySet();
    }

    @Override 
    public void updateState(RobotStateHistory stateHistory){
        getCurrentUpdater().updateState(stateHistory);
    }

    @Override
    public void updateRobotState(){
        getCurrentUpdater().updateState(Robot.stateHistory);
    }

}