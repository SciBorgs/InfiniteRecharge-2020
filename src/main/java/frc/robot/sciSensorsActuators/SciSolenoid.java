package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;

public class SciSolenoid extends DoubleSolenoid {
    private final Enum forwardValue, backwardValue;
    private Enum currentValue;

    public SciSolenoid(int[] ports, Enum forward, Enum backward) {
        super(ports[0], ports[1]);
        this.forwardValue  = forward;
        this.backwardValue = backward;
    }

    public SciSolenoid(int pdpPort, int[] ports, Enum forward, Enum backward) {
        super(pdpPort, ports[0], ports[1]);
        this.forwardValue  = forward;
        this.backwardValue = backward;
    }

    private Value enumToK(Enum e) {
        if(e.equals(this.forwardValue))  {return Value.kForward;}
        if(e.equals(this.backwardValue)) {return Value.kReverse;}
        return Value.kOff;
    }

    public Enum oppositeSciSolenoidValue(Enum e) {
        if(e.equals(this.forwardValue))  {return this.backwardValue;}
        if(e.equals(this.backwardValue)) {return this.forwardValue;}
        return Value.kOff;
    }

    public void set(Enum e) {
        currentValue = e;
        super.set(enumToK(e));
    }

    public void toggle() {
        this.set(oppositeSciSolenoidValue(getValue()));
    }

    public Enum getValue() {return this.currentValue;}

    public Boolean isForward() {
        return this.currentValue.equals(this.forwardValue);
    }

    public Boolean isBackward() {
        return this.currentValue.equals(this.backwardValue);
    }
}