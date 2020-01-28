package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Servo;

public class SciServo extends Servo {
    private static final double ANGLE_RANGE = Math.PI;
    private double minAngle, centerAngle, maxAngle;

    public SciServo(int channel) {
		super(channel);
	}

    public double getMinAngle() { return minAngle; }
    public double getCenterAngle() { return centerAngle; }
    public double getMaxAngle() { return maxAngle; }

    public void setMinAngle(double minAngle) {
        this.minAngle = minAngle;
        this.centerAngle = minAngle + ANGLE_RANGE/2.0;
        this.maxAngle = minAngle + ANGLE_RANGE;
    }

    public void setCenterAngle(double centerAngle) {
        this.minAngle = centerAngle - ANGLE_RANGE/2.0;
        this.centerAngle = centerAngle;
        this.maxAngle = centerAngle + ANGLE_RANGE/2.0;
    }

    public void setMaxAngle(double maxAngle) {
		this.minAngle = maxAngle - ANGLE_RANGE;
        this.centerAngle = maxAngle - ANGLE_RANGE/2.0;
        this.maxAngle = maxAngle;
    }

    public double getAngle() {
        return get() * ANGLE_RANGE + minAngle;
    }

    public void setAngle(double angle) {
        this.set((angle - minAngle) / ANGLE_RANGE);
    }
}