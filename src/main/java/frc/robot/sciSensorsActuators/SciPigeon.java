package frc.robot.sciSensorsActuators;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciPigeon.SciPigeonSD;;

public class SciPigeon extends PigeonIMU implements RobotStateUpdater, SciSensorActuator<SciPigeonSD> {
    public static enum SciPigeonSD {Angle, Pitch, Role}
    public HashMap<SciPigeonSD, SD> sdMap;
    private SciUtils<SciPigeonSD> sciUtils;

    public SciPigeon(TalonSRX talon) {
        super(talon);
        this.sciUtils = new SciUtils<>(this);
        this.sdMap = new HashMap<>();
        Robot.addRobotStateUpdater(this);
    }

    @Override
    public HashMap<SciPigeonSD, SD> getSDMap(){return this.sdMap;}
    @Override 
    public SciUtils<SciPigeonSD> getSciUtils(){return this.sciUtils;}
    @Override 
    public String getDeviceName(){return "Pigeon " + super.getDeviceID();}

    @Override
    public void assignSD(SciPigeonSD pigeonSD, SD sd) {
        this.sciUtils.assignSD(pigeonSD, sd);
    }

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
        this.sciUtils.sciSet(SciPigeonSD.Angle, getAngle());
        this.sciUtils.sciSet(SciPigeonSD.Pitch, getPitch());
        this.sciUtils.sciSet(SciPigeonSD.Role,  getRole());
    }
}