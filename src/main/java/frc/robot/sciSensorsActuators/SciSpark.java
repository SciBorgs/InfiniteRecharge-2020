package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import frc.robot.Utils;

public class SciSpark extends CANSparkMax{

    public final static double DEFAULT_MAX_JERK = 0.1;
    private double gearRatio;
    
    public SciSpark(int port){
        super(port, MotorType.kBrushless);
    }
    public SciSpark(int port, double gearRatio){
        super(port, MotorType.kBrushless);
        this.gearRatio = gearRatio;
    }

    public double getGearRatio(){return this.gearRatio;}
    public void setGearRatio(double gearRatio){this.gearRatio = gearRatio;}

    public double getAngle() {
        // No need for ticks per rev b/c getPosition() returns in # of rotations
        return super.getEncoder().getPosition() * 2 * Math.PI;
    }

    public double getOutputAngle(){
        return getOutputAngle(this.gearRatio);
    }
    public double getOutputAngle(double gearRatio){
        return getAngle() * gearRatio;
    }

    public void set(double speed, double maxJerk){
        super.set(Utils.limitChange(super.get(), speed, maxJerk));
    }
    public void set(double speed){set(speed, DEFAULT_MAX_JERK);}

}