package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;
import frc.robot.dataTypes.BiHashMap;

public class SciSolenoid <ValueType extends Enum<ValueType>> extends DoubleSolenoid {
    private BiHashMap<Value, ValueType> valueMap;

    public SciSolenoid(int[] ports, ValueType forwardValue, ValueType backwardValue, ValueType offValue) {
        this(1, ports, forwardValue, backwardValue, offValue);
    }

    public SciSolenoid(int pdpPort, int[] ports, ValueType forwardValue, ValueType reverseValue, ValueType offValue) {
        super(pdpPort, ports[0], ports[1]);
        this.valueMap = new BiHashMap<Value, ValueType>();
        this.valueMap.put(Value.kForward, forwardValue);
        this.valueMap.put(Value.kReverse, reverseValue);
        this.valueMap.put(Value.kOff,     offValue);
    }
    
    private Value toDoubleSolenoidValue(ValueType e) {
        return valueMap.getBackward(e);
    }

    private ValueType toValueType(Value v){
        return valueMap.getForward(v);
    }

    public ValueType oppositeSciSolenoidValue(ValueType e) {
        return valueMap.getForward(Utils.oppositeDoubleSolenoidValue(valueMap.getBackward(e)));
    }

    public void set(ValueType e) {
        super.set(toDoubleSolenoidValue(e));
    }

    public void toggle() {
        super.set(Utils.oppositeDoubleSolenoidValue(super.get()));
    }

    
    /**
     * get() is deprecated for SciSolenoids. Use getValue() instead.
     * @deprecated
     */
    @Override @Deprecated
    public Value get(){
        throw new RuntimeException("get() is deprecated for SciSolenoids. Use getValue() instead");
    }

    public ValueType getValue() {return toValueType(super.get());}
}