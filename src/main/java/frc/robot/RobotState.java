package frc.robot;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Set;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RobotState {
    
    public enum RS {
        // Position
        X, Y, Angle,

        // Chassis motor values
        LeftWheelAngle, RightWheelAngle, L1WheelAngle, R1WheelAngle, L2WheelAngle, R2WheelAngle,
        LeftSparkVal, RightSparkVal, L1SparkVal, R1SparkVal, L2SparkVal, R2SparkVal,
        LeftSparkVoltage, RightSparkVoltage, L1SparkVoltage, R1SparkVoltage, L2SparkVoltage, R2SparkVoltage,
        LeftSparkCurrent, RightSparkCurrent, L1SparkCurrent, R1SparkCurrent, L2SparkCurrent, R2SparkCurrent,

        // Solenoids
        GearShiftSolenoid,

        // Pneumatics
        PressureSensorVoltage,
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
    public Set<RS> getKeys(){return data.keySet();}

    public static Value intToSolenoidValue(int i){
        switch (i){
            case 1:  return Value.kForward;
            case 0:  return Value.kReverse;
            case -1: return Value.kOff;
        }
        throw new IllegalArgumentException("attempted to convert" + i + " to solenoid value, but v is not -1, 1 or 0");
    }
    public static int solenoidValueToInt(Value v){
        switch (v){
            case kForward: return 1;
            case kOff:     return 0;
            case kReverse: return -1;
            default:       return 0;
        }
    }

    public Value getSolenoidValue(RS rs){
        if ((int) get(rs) == get(rs)){
            return intToSolenoidValue((int) get(rs));
        } else {
            throw new IllegalArgumentException("getSolenoidValue requires an RS bound to an int, but given " + rs + " which is bound to " + get(rs));
        }
    }

    public RobotState copy(){
        return new RobotState((Hashtable<RS, Double>) data.clone());
    }
    public RobotState copyIntoNew(RS rs, double val){
        RobotState newRobotState = copy();
        newRobotState.set(rs, val);
        return newRobotState;
    }

    public void incorporateOtherState(RobotState otherState){
        // Will use any values from the other state
        // But will keep any values it has from keys that that the other state lacks 
        incorporateOtherState(otherState, otherState.getKeys());
    }
    public void incorporateOtherState(RobotState otherState, Iterable<RS> toTake) {
        // Will use any the toTake values from otherState
        for (RS rs : toTake) {set(rs, otherState.get(rs));}
    }
    public RobotState incorporateIntoNew(RobotState otherState, Iterable<RS> toTake){
        RobotState newRobotState = copy();
        newRobotState.incorporateOtherState(otherState, toTake);
        return newRobotState;
    }
    public RobotState incorporateIntoNew(RobotState otherState){
        return incorporateIntoNew(otherState, otherState.getKeys());
    }

    public void cutDownTo(ArrayList<RS> rsToInclude){
        // Will get rid of all keys not in rsToInclude
        for(RS rs : getKeys()){
            if (!rsToInclude.contains(rs)){remove(rs);}
        }
    }

}