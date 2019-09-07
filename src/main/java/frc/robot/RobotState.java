package frc.robot;

import frc.robot.stateEstimation.*;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Set;

public class RobotState {
    
    public enum RS {
        Xpos, Ypos, Angle
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
        for(RS rs : otherState.keys()){
            set(rs, otherState.get(rs));
        }
    }

    public void cutDownTo(ArrayList<RS> rsToInclude){
        for(RS rs : keys()){
            if (!rsToInclude.contains(rs)){
                remove(rs);
            }
        }
    }

}