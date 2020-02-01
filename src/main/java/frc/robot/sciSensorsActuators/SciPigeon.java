package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class SciPigeon extends PigeonIMU {

    public SciPigeon(TalonSRX talon) {
        super(talon);
    }

    public PigeonIMU getPigeonIMU() {return this;}

    private double[] yawPitchRole(){
        double[] yawPitchRoll = new double[3];
        super.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll;
    }

    public double getAngle(){return Math.toRadians(yawPitchRole()[0]);}
    public double getPitch(){return Math.toRadians(yawPitchRole()[1]);}
    public double getRole() {return Math.toRadians(yawPitchRole()[2]);}
    
    public void setAngle(double angle){
        super.setYaw(angle);
    }
}