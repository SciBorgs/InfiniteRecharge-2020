package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciThroughBoreEncoder extends DutyCycleEncoder {
    public SciThroughBoreEncoder(int port) { // NOTE: Must be connected to DIO NOT PWM
        super(port);
        reset();
    }

    public double getRadians() {
        return get() * 2 * Math.PI;
    }
}