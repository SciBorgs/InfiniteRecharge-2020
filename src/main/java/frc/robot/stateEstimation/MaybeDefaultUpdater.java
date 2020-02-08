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

    @Override
    public Hashtable<SD, Double> getStdDevs(){
        Hashtable<SD, Double> stdDevs = this.defaultUpdater.getStdDevs();
        if (this.maybeUpdater.canUpdate()){
            for(SD sd : this.maybeUpdater.getStdDevs().keySet()){
                stdDevs.put(sd, this.maybeUpdater.getStdDevs().get(sd));
            }
        }
        return stdDevs;
    }

    @Override
    public Set<SD> getSDs(){
        return getStdDevs().keySet();
    }

    @Override
    public void updateState(RobotStateHistory stateHistory){
        this.defaultUpdater.updateState(stateHistory);
        if (this.maybeUpdater.canUpdate()){
            //System.out.println("calling limelight update");
            this.maybeUpdater.updateState(stateHistory);
        }
    }

    @Override
    public void updateRobotState(){
        //System.out.println("udpating robot state");
        this.updateState(Robot.stateHistory);
    }

}