package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.dataTypes.BiHashMap;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;

import java.util.Optional;


public class SciSolenoid <ValueType extends Enum<ValueType>> extends DoubleSolenoid implements RobotStateUpdater {

    private Class valueTypeClass;
    private ValueType defaultValue;
    public final static BiHashMap<Value, Double> SOLENOID_MAPPING;
    static {
        SOLENOID_MAPPING=new BiHashMap<>();
        SOLENOID_MAPPING.put(Value.kForward, 1.0);
        SOLENOID_MAPPING.put(Value.kOff,     0.0);
        SOLENOID_MAPPING.put(Value.kReverse,-1.0);
    }
    private BiHashMap<ValueType, Value> valueMap;
    private BiHashMap<ValueType, Double> valueDoubleMap;
    public Optional<SD> valueSD;
    private boolean printValues;

    public SciSolenoid(int[] ports, ValueType forwardValue, ValueType backwardValue, ValueType offValue, ValueType defaultValue) {
        this(1, ports, forwardValue, backwardValue, offValue, defaultValue);
    }

    public SciSolenoid(int pdpPort, int[] ports, ValueType forwardValue, ValueType reverseValue, ValueType offValue, ValueType defaultValue) {
        super(pdpPort, ports[0], ports[1]);
        this.valueMap = new BiHashMap<ValueType, Value>();
        this.valueMap.put(forwardValue, Value.kForward);
        this.valueMap.put(reverseValue, Value.kReverse);
        this.valueMap.put(offValue,      Value.kOff);
        for (ValueType valueType : valueMap.keySet()){
            Value value = valueMap.getForward(valueType);
            this.valueDoubleMap.put(valueType, SOLENOID_MAPPING.getForward(value));
        }
        this.valueSD = Optional.empty();
        this.valueTypeClass = forwardValue.getClass();
        this.printValues = false;
        this.defaultValue = defaultValue;

        Robot.addRobotStateUpdater(this);
    }
    
    private Value toDoubleSolenoidValue(ValueType e) {
        return valueMap.getForward(e);
    }

    private ValueType toValueType(Value v){
        return valueMap.getBackward(v);
    }

    public ValueType oppositeSciSolenoidValue(ValueType e) {
        return (toDoubleSolenoidValue(e) == Value.kOff) 
            ? defaultValue 
            : valueMap.getBackward(Utils.oppositeDoubleSolenoidValue(valueMap.getForward(e)));
    }

    public void set(ValueType e) {
        super.set(toDoubleSolenoidValue(e));
    }

    public void toggle() {
        set(oppositeSciSolenoidValue(valueMap.getBackward(super.get())));
    }

    public void printValues()    {this.printValues = true;}
    public void dontPrintValues(){this.printValues = false;}

    /**
     * get() is deprecated for SciSolenoids. Use getValue() instead.
     * @deprecated
     */
    @Override @Deprecated
    public Value get(){
        throw new RuntimeException("get() is deprecated for SciSolenoids. Use getValue() instead");
    }

    public ValueType getValue() {return toValueType(super.get());}

    public void assignValueSD(SD valueSD){this.valueSD = Optional.of(valueSD);}

    public void updateRobotState(){
        Robot.optionalMappedSet(this.valueDoubleMap, this.valueSD, getValue());
        if (this.printValues){
            System.out.println("SciSolenoid<" + this.valueTypeClass + "> value: " + getValue());
        }
    }
}
