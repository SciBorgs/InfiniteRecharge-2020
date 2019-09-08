package frc.robot;

import frc.robot.stateEstimation.*;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Set;

public class RobotState {
    
    public enum RS {
        X, Y, Angle, 
        lfWheelAngle, rfWheelAngle
    }

    private Hashtable<RS, Double> data;

    public RobotState() {
        this.data = new Hashtable<>();
    }
    public RobotState(Hashtable<RS, Double> data) {
        this.data = data;
    }

    public double get(RS rs)      {return this.data.get(rs);}
    public void   set(RS rs, double val) {this.data.put(rs, val);}
    public void   remove(RS rs)          {this.data.remove(rs);}

    public RobotState copy(){
        return new RobotState((Hashtable<RS, Double>) data.clone());
    }
    public RobotState setCopy(RS rs, double val){
        RobotState newRobotState = copy();
        newRobotState.set(rs, val);
        return newRobotState;
    }
    public Set<RS> keys(){return data.keySet();}

    public void incorporateOtherState(RobotState otherState){
        // Will use any values from the other state
        // But will keep any values it has from keys that that the other state lacks 
        incorporateOtherState(otherState, otherState.keys());
    }
    public void incorporateOtherState(RobotState otherState, Iterable<RS> toTake) {
        // Will use any the toTake values from otherState
        for (RS rs : toTake) {
            set(rs, otherState.get(rs));
        }
    }
    public RobotState incorporateIntoNew(RobotState otherState, Iterable<RS> toTake){
        RobotState newRobotState = copy();
        newRobotState.incorporateOtherState(otherState, toTake);
        return newRobotState;
    }
    public RobotState incorporateIntoNew(RobotState otherState){
        return incorporateIntoNew(otherState, otherState.keys());
    }

    public void cutDownTo(ArrayList<RS> rsToInclude){
        // Will get rid of all keys not in rsToInclude
        for(RS rs : keys()){
            if (!rsToInclude.contains(rs)){
                remove(rs);
            }
        }
    }

}