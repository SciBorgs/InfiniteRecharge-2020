package frc.robot.sciSensorsActuators;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;

public class SciPigeon extends PigeonIMU implements RobotStateUpdater {
    public Optional<SD> angleSD, pitchSD, roleSD;

    public SciPigeon(TalonSRX talon) {
        super(talon);
        Robot.addRobotStateUpdater(this);
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

    public void updateRobotState(){
        Robot.optionalSet(this.angleSD, getAngle());
        Robot.optionalSet(this.pitchSD, getPitch());
        Robot.optionalSet(this.roleSD,  getRole());
    }

    public void assignAngleSD(SD angleSD) {this.angleSD = Optional.of(angleSD);}
    public void assignPitchSD(SD pitchSD) {this.pitchSD = Optional.of(pitchSD);}
    public void assignRoleSD (SD roleSD)  {this.roleSD  = Optional.of(roleSD);}
}