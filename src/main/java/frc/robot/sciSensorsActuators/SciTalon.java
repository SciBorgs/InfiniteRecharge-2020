package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;

public class SciTalon extends TalonSRX{

    public final static double DEFAULT_MAX_JERK = 0.1;
    public static final double ENC_TICKS_PER_REV = 4096;
    public double gearRatio;
    
    public SciTalon(int port){super(port);}
    public SciTalon(int port, double gearRatio){
        super(port);
        this.gearRatio = gearRatio;
    }

    public double getGearRatio(){return this.gearRatio;}
    public void setGearRatio(double gearRatio){this.gearRatio = gearRatio;}

    public double getAngle() {
        return super.getSensorCollection().getQuadraturePosition() / ENC_TICKS_PER_REV * 2 * Math.PI;
    }

    public double getOutputAngle(){
        return getOutputAngle(this.gearRatio);
    }
    public double getOutputAngle(double gearRatio){
        return getAngle() * gearRatio;
    }

    public void set(double speed, double maxJerk){
        super.set(ControlMode.PercentOutput, Utils.limitChange(super.getMotorOutputPercent(), speed, maxJerk));
    }
    public void set(double speed){set(speed, DEFAULT_MAX_JERK);}

}