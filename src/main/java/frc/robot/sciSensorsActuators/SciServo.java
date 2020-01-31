package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Servo;

public class SciServo extends Servo {
    private static final double ANGLE_RANGE = Math.PI;
    private double minAngle;

    public SciServo(int channel) {
        super(channel);
    }

    public double getMinAngle()    { return this.minAngle; }
    public double getCenterAngle() { return this.minAngle + ANGLE_RANGE/2; }
    public double getMaxAngle()    { return this.minAngle + ANGLE_RANGE; }

    public void setMinAngle(double angle)    { this.minAngle = angle; }

    public void setCenterAngle(double angle) { this.minAngle = angle - ANGLE_RANGE/2.0; }

    public void setMaxAngle(double angle)    { this.minAngle = angle - ANGLE_RANGE; }

    @Override
    public double getAngle() {
        return Math.toRadians(super.getAngle()) + this.minAngle;
    }

    @Override
    public void setAngle(double angle) {
        super.setAngle(Math.toDegrees(angle - this.minAngle));
    }
}
