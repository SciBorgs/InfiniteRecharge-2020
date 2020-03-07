package frc.robot.sciSensorsActuators;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciPigeon.SciPigeonSD;;

public class SciPigeon extends PigeonIMU implements RobotStateUpdater, SciSensorActuator<SciPigeonSD> {
    public static enum SciPigeonSD {Angle, Pitch, Role}
    public HashMap<SciPigeonSD, SD> sdMap;

    public SciPigeon(int id) {
        super(id);
        this.sdMap = new HashMap<>();
        automateStateUpdating();
    }

    @Override
    public HashMap<SciPigeonSD, SD> getSDMap(){return this.sdMap;}
    @Override 
    public String getDeviceName(){return "Pigeon " + super.getDeviceID();}

    @Override
    public boolean ignore(){return sciIgnore(super.getDeviceID());}

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

    public void updateRobotState(){
        sciSet(SciPigeonSD.Angle, getAngle());
        sciSet(SciPigeonSD.Pitch, getPitch());
        sciSet(SciPigeonSD.Role,  getRole());
    }
}