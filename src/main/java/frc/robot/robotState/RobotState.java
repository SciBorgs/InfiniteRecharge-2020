package frc.robot.robotState;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.dataTypes.BiHashMap;

public class RobotState {    
    // SD = State Dimension
    public enum SD {
        // Position
        X, Y, PigeonAngle, Angle,

        // Chassis motor values
        LeftWheelAngle, RightWheelAngle,
        LeftSparkVal, RightSparkVal,
        LeftCurrentVal, RightCurrentVal,
        
        // Solenoids
        GearShiftSolenoid,

        // Pneumatics
        PressureSensorVoltage,
    }
    
    private Hashtable<SD, Double> data;

    public final static BiHashMap<Boolean, Double> BOOLEAN_MAPPING;

    static {
        BOOLEAN_MAPPING = new BiHashMap<>();
        BOOLEAN_MAPPING.put(true,  1.0);
        BOOLEAN_MAPPING.put(false, 0.0);
        RobotState.SD sd = SD.X;

    }

    public RobotState() {
        this(new Hashtable<>());
    }

    public RobotState(Hashtable<SD, Double> data) {
        this.data = data;
    }

    public boolean contains(SD sd) {return this.data.containsKey(sd);}
    public double get(SD sd)      {return this.data.get(sd);}
    public void   set(SD sd, double val) {this.data.put(sd, val);}
    public void   remove(SD sd)          {this.data.remove(sd);}
    public Set<SD> getKeys(){return data.keySet();}

    public void optionalSet(Optional<SD> optionalSD, double v){
        if (optionalSD.isPresent()){set(optionalSD.get(), v);}
    }
    public<K> void optionalMappedSet(BiHashMap<K, Double> biMap, Optional<SD> optionalSD, K v){
        if (optionalSD.isPresent()){setMapped(biMap, optionalSD.get(), v);}
    }

    public<K> void setMapped(BiHashMap<K, Double> biMap, SD sd, K key) {
        this.data.put(sd, biMap.getForward(key));
    }

    public<K> K getMapped(BiHashMap<K, Double> biMap, SD sd) {
        return biMap.getBackward(get(sd));
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
