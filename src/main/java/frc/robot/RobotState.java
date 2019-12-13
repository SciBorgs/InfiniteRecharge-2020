package frc.robot;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Set;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RobotState {
    
    // SD = State Dimension
    public enum SD {
        // Position
        X, Y, Angle, PigeonAngle,

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

    private Hashtable<SD, Double> data;

    public RobotState() {
        this.data = new Hashtable<>();
    }
    public RobotState(Hashtable<SD, Double> data) {
        this.data = data;
    }

    public double get(SD sd)      {return this.data.get(sd);}
    public void   set(SD sd, double val) {this.data.put(sd, val);}
    public void   remove(SD sd)          {this.data.remove(sd);}
    public Set<SD> getKeys(){return data.keySet();}

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

    public Value getSolenoidValue(SD sd){
        // if ((int) get(sd) == get(sd)){
            return intToSolenoidValue((int) get(sd));
        //} else {
        //    throw new IllegalArgumentException("getSolenoidValue requires an SD bound to an int, but given " + sd + " which is bound to " + get(sd));
        //}
    }

    public RobotState copy(){
        return new RobotState((Hashtable<SD, Double>) data.clone());
    }
    public RobotState copyIntoNew(SD sd, double val){
        RobotState newRobotState = copy();
        newRobotState.set(sd, val);
        return newRobotState;
    }

    public void incorporateOtherState(RobotState otherState){
        // Will use any values from the other state
        // But will keep any values it has from keys that that the other state lacks 
        incorporateOtherState(otherState, otherState.getKeys());
    }
    public void incorporateOtherState(RobotState otherState, Iterable<SD> toTake) {
        // Will use any the toTake values from otherState
        for (SD sd : toTake) {set(sd, otherState.get(sd));}
    }
    public RobotState incorporateIntoNew(RobotState otherState, Iterable<SD> toTake){
        RobotState newRobotState = copy();
        newRobotState.incorporateOtherState(otherState, toTake);
        return newRobotState;
    }
    public RobotState incorporateIntoNew(RobotState otherState){
        return incorporateIntoNew(otherState, otherState.getKeys());
    }

    public void cutDownTo(ArrayList<SD> sdToInclude){
        // Will get rid of all keys not in sdToInclude
        for(SD sd : getKeys()){
            if (!sdToInclude.contains(sd)){remove(sd);}
        }
    }

}