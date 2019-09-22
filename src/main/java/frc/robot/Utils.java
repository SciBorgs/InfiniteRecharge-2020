package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotState.RS;
import frc.robot.helpers.Pair;

import java.util.*;
import java.util.Collections;
import java.util.stream.Collectors;
import java.util.stream.Stream;

// FILE HAS NOT BEEN CLEANED UP //
public class Utils{

    private static Random r = new Random();

    public static double METERS_TO_INCHES = 39.37;

    public static double metersToInches(double meters){return meters * METERS_TO_INCHES;}
    public static double inchesToMeters(double inches){return inches / METERS_TO_INCHES;}

    public static<T> T last(ArrayList<T> arr) {return arr.get(arr.size() - 1);}

    public static int signOf(double value){
        if (value == 0){
            return 0;
        } else {
            return (int) (Math.abs(value) / value);
        }
    }

    public static void trimIf(ArrayList<Double> arr, int maxSize) {
        // Trims an array down to a max size, starting from the start
        while (maxSize < arr.size()){arr.remove(0);}
    }

    public static boolean inRange(double n1, double n2, double error){
        return Math.abs(n1 - n2) < error;
    }

    public static boolean inRange(ArrayList<Double> arr, double error){
        return inRange(Collections.max(arr), Collections.min(arr), error);
    }

    public static double averageRange(ArrayList<Double> arr) {
        return (last(arr) - arr.get(0)) / arr.size();
    }

    public static void trimAdd(ArrayList<Double> arr, double val, int maxSize) {
        // Adds a value to the array and then trims it to a max size
        arr.add(val);
        trimIf(arr, maxSize);
    }

    public static double limitOutput(double output, double max){
        if (output > max) {
            return max;
        } else if (output < - max) {
            return -max;
        } else {
            return output;
        }
    }

    public static void setTalon(TalonSRX talon, double speed){
        talon.set(ControlMode.PercentOutput, speed);
    }

    public static int boolToInt(boolean b){
        if (b) {
            return 1;
        } else {
            return 0;
        }
    }

    public static HashSet<String> arrayListToHashset(ArrayList<String> values){
        HashSet<String> end = new HashSet<String>();
        for (String val : values){
            end.add(val);
        }
        return end;
    }

    public static Hashtable<String,String> hatshtableDataToString(Hashtable<String,Object> hashTable){
        Hashtable<String,String> end = new Hashtable<String,String>();
        for (String key : hashTable.keySet()){
            end.put(key,hashTable.get(key).toString());
        }
        return end;
    }

    // Generic function to merge 2 arrays of same type in Java
    public static<T> T[] combineArray(T[] arr1, T[] arr2) {
	    T[] result = Arrays.copyOf(arr1, arr1.length + arr2.length);
	    System.arraycopy(arr2, 0, result, arr1.length, arr2.length);
	    return result;
    }

    public static Value oppositeDoubleSolenoidValue(Value val){
        switch (val) {
            case kForward: return Value.kReverse;
            case kReverse: return Value.kForward;
            default:       return Value.kOff;
        }

    }

    public static void toggleDoubleSolenoid(DoubleSolenoid doubleSolenoid){
        doubleSolenoid.set(oppositeDoubleSolenoidValue(doubleSolenoid.get()));
    }

    public static boolean oppositeDigitalOutput(boolean bool){return !bool;}

    public static void toggleDigitalOutput(DigitalOutput digitalOutput){
        oppositeDigitalOutput(digitalOutput.get());
    }

    public static DoubleSolenoid newDoubleSolenoid(int[] ports){
        return new DoubleSolenoid(ports[0], ports[1]);
    }
    public static DoubleSolenoid newDoubleSolenoid(int pdpPort, int[] ports){
        return new DoubleSolenoid(pdpPort, ports[0], ports[1]);
    }

    public static double generateGaussian(double mean, double stdDev){
        return r.nextGaussian() * stdDev + mean;
    }

    public static ArrayList<Double> cummSums(ArrayList<Double> arr){
        ArrayList<Double> ans = new ArrayList<>();
        double sum = 0;
        for(double n : arr){
            sum += n;
            ans.add(sum);
        }
        return ans;
    }

    public static ArrayList<Double> randomArrayList(int length, double min, double max){
        ArrayList<Double> arr = new ArrayList<>();
        for(int _i = 0; _i < length; _i++){
            arr.add(r.nextDouble() * (max - min) + min);
        }
        return arr;
    }

    public static Comparator<Double> ascendingDoubleComparator = Comparator.comparingDouble(d -> d);

    public static boolean inBounds(double v, Pair<Double,Double> bounds){
        double min = Math.min(bounds.first, bounds.second);
        double max = Math.max(bounds.first, bounds.second);
        return v >= min && v <= max;
    }

    public static<T> ArrayList<T> toArrayList(Iterable<T> iterable){
        ArrayList<T> arrayList = new ArrayList<>();
        for(T el : iterable){arrayList.add(el);}
        return arrayList;
    }
    public static<T> ArrayList<T> toArrayList(T[] arr){
        ArrayList<T> arrayList = new ArrayList<>();
        for(T el : arr){arrayList.add(el);}
        return arrayList;
    }
    public static<T> ArrayList<T> toArrayList(Stream<T> stream){
        ArrayList<T> arrayList = new ArrayList<>();
        for(T el : stream.collect(Collectors.toList())){arrayList.add(el);}
        return arrayList;
    }

    public static double sumArrayList(List<Double> l){
        return l.stream().mapToDouble(a -> a).sum();
    }

    public static void addNoise(RobotState state, Hashtable<RS, Double> stdDevs){
        for (RS rs : stdDevs.keySet()) {
            // generate gaussian creates noise
            state.set(rs, Utils.generateGaussian(state.get(rs), stdDevs.get(rs)));
        }
    }
}