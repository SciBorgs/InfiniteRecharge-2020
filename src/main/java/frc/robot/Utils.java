package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.robotState.RobotState.SD;
import frc.robot.robotState.*;
import frc.robot.dataTypes.Pair;

import java.util.*;
import java.util.Collections;
import java.util.stream.Collectors;
import java.util.stream.Stream;

// FILE HAS NOT BEEN CLEANED UP //
public class Utils{

    private static Random r = new Random();

    public static final double METERS_TO_INCHES = 39.37;
    public static final double SECONDS_PER_MINUTE = 60;
    // essnetially an error that we're willing to have, probably due to floating point rounding
    public static final double EPSILON = 1e-6;

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

    public static boolean inRange(double n1, double n2, double error){
        return Math.abs(n1 - n2) < error;
    }

    public static boolean inRange(ArrayList<Double> arr, double error){
        return inRange(Collections.max(arr), Collections.min(arr), error);
    }

    public static double averageRange(ArrayList<Double> arr) {
        return (last(arr) - arr.get(0)) / arr.size();
    }

    // if a number isn't in the range, you keep on adding the difference (max - min) until it is in the range
    // for example, you can use it to find an angle that is overlapping within a range, such as -180 and 180
    public static double bringInRange(double val, double min, double max) {
        return mod(val - min, max - min) + min;
    }

    // if it isn't between max or min, it will bring it down to max, or up to min
    public static double limitOutput(double output, double max, double min){
        return Math.min(Math.max(output, min), max);
    }

    public static double limitOutput(double output, double max){
        return limitOutput(output, max, 0 - max);
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

    // Generic function to merge 2 arrays of same type 
    public static<T> T[] combineArray(T[] arr1, T[] arr2) {
	    T[] result = Arrays.copyOf(arr1, arr1.length + arr2.length);
	    System.arraycopy(arr2, 0, result, arr1.length, arr2.length);
	    return result;
    }

    // Deep copy (no referencing) of an arraylist
    public static<T> void deepCopy(ArrayList<T> arrToCopy, ArrayList<T> newArr) {
        for(int i = 0; i < arrToCopy.size() - 1; i++) {
            newArr.add(arrToCopy.get(i));
        }
    }

    public static Value oppositeDoubleSolenoidValue(Value val){
        switch (val) {
            case kForward: return Value.kReverse;
            case kReverse: return Value.kForward;
            default:       return Value.kOff;
        }
    }

    public static void toggleDigitalOutput(DigitalOutput digitalOutput){
        digitalOutput.set(!digitalOutput.get());
    }

    // given a mean and a stddev, generates a random number
    public static double generateGaussian(double mean, double stdDev){
        return r.nextGaussian() * stdDev + mean;
    }

    // get's all the cummulative summs of an arraylist
    // for example: [1,3,5,2] -> [1,1+3,1+3+5,1+3+5+2] -> [1,4,9,11]
    public static ArrayList<Double> cummSums(Iterable<Double> arr){
        ArrayList<Double> ans = new ArrayList<>();
        double sum = 0;
        for(double n : arr){
            sum += n;
            ans.add(sum);
        }
        return ans;
    }

    // Generates an arraylist given a length, with random numbers between min and max
    public static ArrayList<Double> randomArrayList(int length, double min, double max){
        ArrayList<Double> arr = new ArrayList<>();
        for(int _i = 0; _i < length; _i++){
            arr.add(r.nextDouble() * (max - min) + min);
        }
        return arr;
    }

    // Just so that we get the ascencion vs descnsion correctly
    public static Comparator<Double> ascendingDoubleComparator = Comparator.comparingDouble(d -> d);

    public static boolean inBounds(double v, Pair<Double,Double> bounds){
        double min = Math.min(bounds.first, bounds.second);
        double max = Math.max(bounds.first, bounds.second);
        return v >= min && v <= max;
    }

    public static boolean impreciseEquals(double d1, double d2) {
        return inRange(d1, d2, EPSILON);
    }

    public static<T> ArrayList<T> toArrayList(Iterable<T> iterable){
        ArrayList<T> arrayList = new ArrayList<>();
        for(T el : iterable){arrayList.add(el);}
        return arrayList;
    }
    public static<T> Stream<T> toStream(Iterable<T> iterable){
        return toArrayList(iterable).stream();
    }
    public static double sumArrayList(List<Double> l){
        return l.stream().mapToDouble(a -> a).sum();
    }

    public static void addNoise(RobotState state, Hashtable<SD, Double> stdDevs){
        for (SD sd : stdDevs.keySet()) {
            // generate gaussian to creates noise
            state.set(sd, Utils.generateGaussian(state.get(sd), stdDevs.get(sd)));
        }
    }

    public static double limitChange(double oldN, double newN, double maxChange) {
        // Makes sure that the change in input for a motor is not more than maxJerk
        return limitOutput(newN, oldN + maxChange, oldN - maxChange);
    }

    public static double max(double... numbers) {
        double max = numbers[0];
        for (double n : numbers) {
            if (n > max) {
                max = n;
            }
        }
        return max;
    }

    public static double min(double... numbers) {
        double min = numbers[0];
        for (double n : numbers) {
            if (n < min) {
                min = n;
            }
        }
        return min;
    }

    public static double mod(double num1, double num2) {
        return (num1 % num2 + num2) % num2;
    }

    public static<T> ArrayList<T> initArrayList(T... args){
        ArrayList<T> arrayList = new ArrayList<T>();
        for (T el : args){arrayList.add(el);}
        return arrayList;
    }
    public static<T> ArrayList<T> optionalInitArrayList(Optional<T>... args){
        ArrayList<T> arrayList = new ArrayList<T>();
        for (Optional<T> el : args){
            if (el.isPresent()){arrayList.add(el.get());}
        }
        return arrayList;
    }

}
