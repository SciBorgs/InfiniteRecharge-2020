package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;

public class SciSolenoid extends DoubleSolenoid {
    
    private final Value FORWARD;
    private final Value BACKWARD;

    public SciSolenoid(int[] ports, Value forward){
        super(ports[0], ports[1]);
        FORWARD = forward;
        BACKWARD = Utils.oppositeDoubleSolenoidValue(FORWARD);
    }
    public SciSolenoid(int pdpPort, int[] ports, Value forward){
        super(pdpPort, ports[0], ports[1]);
        FORWARD = forward;
        BACKWARD = Utils.oppositeDoubleSolenoidValue(FORWARD);
    }

    public void goForward(){
        super.set(FORWARD);
    }
    public void goBackward() {
        super.set(BACKWARD);
    }

    public void toggle() {
        super.set(Utils.oppositeDoubleSolenoidValue(super.get()));
    }

    public Boolean isForward() {
        return super.get() == FORWARD;
    }
    public Boolean isBackward() {
        return super.get() == BACKWARD;
    }
}