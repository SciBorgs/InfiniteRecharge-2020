package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import frc.robot.Utils;

public class SciSpark extends CANSparkMax {

    public final static double DEFAULT_MAX_JERK = 0.1;
    private double gearRatio;

    public SciSpark(int port) {
        this(port, 1);
    }

    public SciSpark(int port, double gearRatio) {
        super(port, MotorType.kBrushless);
        setWheelAngle(0);
        setGearRatio(gearRatio);
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public void setGearRatio(double gearRatio) {
        double currentAngle = getWheelAngle();
        this.gearRatio = gearRatio;
        super.getEncoder().setPositionConversionFactor(this.gearRatio * 2 * Math.PI);
        // W/o this when we changed gear ratios, it would weirdly change the angle
        // For instance, if you have a gear ratio of 1/5 and your getPosition() is 20 radians
        //  and then you change the gear ratio to 1/10, your getPosition() will go to 10 radians
        setWheelAngle(currentAngle);
    }

    public void setWheelAngle(double angle) {
        super.getEncoder().setPosition(angle / super.getEncoder().getPositionConversionFactor());
    }

    public double getWheelAngle() {
        return super.getEncoder().getPosition();
    }

    public void set(double speed, double maxJerk) {
        super.set(Utils.limitChange(super.get(), speed, maxJerk));
    }

    public void set(double speed) {
        set(speed, DEFAULT_MAX_JERK);
    }

}