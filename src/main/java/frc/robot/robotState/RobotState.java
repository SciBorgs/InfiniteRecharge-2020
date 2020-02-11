package frc.robot.robotState;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;
import frc.robot.dataTypes.BiHashMap;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState.SD;

public class RobotState implements Iterable<Pair<SD, Double>>{
    // SD = State Dimension
    public enum SD {
        // Position
        X, Y, PigeonAngle, Angle,

        // Chassis direct motor values
        LeftWheelAngle, LeftWheelVal, LeftWheelCurrent, LeftWheelSpeed,
        RightWheelAngle, RightWheelVal, RightWheelCurrent, RightWheelSpeed,
        // Chassis calculated motor values
        LeftWheelAccel, LeftWheelJerk, LeftWheelSnap,
        RightWheelAccel, RightWheelJerk, RightWheelSnap,

        // Solenoids
        GearShiftSolenoid,

        // Pneumatics
        PressureSensorVoltage,

        Time
    }
    
    private HashMap<SD, Double> data;

    public final static BiHashMap<Boolean, Double> BOOLEAN_MAPPING;

    static {
        BOOLEAN_MAPPING = new BiHashMap<>();
        BOOLEAN_MAPPING.put(true,  1.0);
        BOOLEAN_MAPPING.put(false, 0.0);

    }

    public RobotState() {
        this(new HashMap<>());
    }

    public RobotState(HashMap<SD, Double> data) {
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
        return new RobotState((HashMap<SD, Double>) data.clone());
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

    public void cutDownTo(Iterable<SD> sdToInclude){
        // Will get rid of all keys not in sdToInclude
        HashSet<SD> sdSet = Utils.toStream(sdToInclude).collect(Collectors.toCollection(HashSet::new));
        ArrayList<SD> sdsToRemove = new ArrayList<>();
        for(SD sd : getKeys()){
            if (!sdSet.contains(sd)){sdsToRemove.add(sd);}
        }

        for(SD sd: sdsToRemove){
            remove(sd);
        }
    }
    public RobotState cutDownIntoNew(Iterable<SD> sdToInclude){
        RobotState newState = this.copy();
        newState.cutDownTo(sdToInclude);
        return newState;
    }

    @Override 
    public Iterator<Pair<SD,Double>> iterator(){
        return new RobotStateIterator(this.data);
    }
  
}

class RobotStateIterator implements Iterator<Pair<SD,Double>> {

    private HashMap<SD,Double> data;
    private SD[] keys;
    private int indexOn;

    public RobotStateIterator(HashMap<SD, Double> data) {
        this.data = (HashMap<SD, Double>) data.clone();
        this.keys = new SD[this.data.keySet().size()];
        this.keys = this.data.keySet().toArray(this.keys);
        this.indexOn = 0;
    }

    @Override
    public boolean hasNext() {
        return indexOn < keys.length;
    }

    @Override
    public Pair<SD, Double> next() {
        SD key = this.keys[indexOn];
        Pair<SD, Double> pair = new Pair<>(key, this.data.get(key));
        this.indexOn++;
        return pair;
    }

    @Override
    public void remove() {
        this.data.remove(this.keys[indexOn]);
        this.indexOn++;
    }
}