package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;

public class SciSolenoid <ValueType extends Enum> extends DoubleSolenoid {
    private ValueType forwardValue, backwardValue, offValue;

    public SciSolenoid(int[] ports, ValueType forward, ValueType backward, ValueType off) {
        this(1, ports, forward, backward, off);
    }

    public SciSolenoid(int pdpPort, int[] ports, ValueType forwardValue, ValueType backwardValue, ValueType offValue) {
        super(pdpPort, ports[0], ports[1]);
        this.forwardValue  = forwardValue;
        this.backwardValue = backwardValue;
        this.offValue      = offValue;
    }
    
    private Value toDoubleSolenoidValue(ValueType e) {
        if(e.equals(this.forwardValue))  {return Value.kForward;}
        if(e.equals(this.backwardValue)) {return Value.kReverse;}
        return Value.kOff;
    }
    private ValueType toValueType(Value v){
        if(v.equals(Value.kForward)) {return this.forwardValue;}
        if(v.equals(Value.kReverse)) {return this.backwardValue;}
        return this.offValue;

    }

    public ValueType oppositeSciSolenoidValue(ValueType e) {
        if(e.equals(this.forwardValue))  {return this.backwardValue;}
        if(e.equals(this.backwardValue)) {return this.forwardValue;}
        return null;
    }

    public void set(ValueType e) {
        super.set(toDoubleSolenoidValue(e));
    }

    public void toggle() {
        this.set(oppositeSciSolenoidValue(getValue()));
    }

    public ValueType getValue() {return toValueType(super.get());}
}