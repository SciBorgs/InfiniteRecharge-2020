package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciThroughBoreEncoder extends DutyCycleEncoder {
    private double offset;

    public SciThroughBoreEncoder(int port) { // NOTE: Must be connected to DIO NOT PWM
        super(port);
        this.offset = 0;
        //super.setDistancePerRotation(2 * Math.PI);
    }

    public void setOffset(double offset){this.offset = offset;}
    public void setAngle(double angle)  {this.offset = angle - getDistance();}

    public double getRadians() {
        return getDistance() + this.offset;
    }
}