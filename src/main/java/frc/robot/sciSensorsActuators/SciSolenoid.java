package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;

public class SciSolenoid <ValueType extends Enum> extends DoubleSolenoid {
    private ValueType forwardValue, backwardValue, currentValue;

    public SciSolenoid(int[] ports, ValueType forward, ValueType backward) {
        super(ports[0], ports[1]);
        setValues(forward, backward);
    }

    public SciSolenoid(int pdpPort, int[] ports, ValueType forward, ValueType backward) {
        super(pdpPort, ports[0], ports[1]);
        setValues(forward, backward);
    }

    private Value enumToK(ValueType e) {
        if(e.equals(this.forwardValue))  {return Value.kForward;}
        if(e.equals(this.backwardValue)) {return Value.kReverse;}
        return Value.kOff;
    }

    public ValueType oppositeSciSolenoidValue(ValueType e) {
        if(e.equals(this.forwardValue))  {return this.backwardValue;}
        if(e.equals(this.backwardValue)) {return this.forwardValue;}
        return null;
    }

    public void set(ValueType e) {
        currentValue = e;
        super.set(enumToK(e));
    }

    public void toggle() {
        this.set(oppositeSciSolenoidValue(getValue()));
    }

    public ValueType getValue() {return this.currentValue;}

    public boolean isForward() {
        return this.currentValue == this.forwardValue;
    }

    public boolean isBackward() {
        return this.currentValue == this.backwardValue;
    }

    public void setValues(ValueType f, ValueType b) {
        this.forwardValue = f;
        this.backwardValue = b;
    }
}