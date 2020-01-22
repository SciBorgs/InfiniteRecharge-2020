package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    private SciSpark telescopeSpark;
    private SciTalon stringTalon1, stringTalon2;
    private SciSpark shiftMotor;

    private final double CASCADE_GEAR_RATIO = 1; // 

    private final double TELESCOPING_GEAR_RATIO = 5;
    private final double TELESCOPING_UP_SPEED   = 3;
    private final double TELESCOPING_DOWN_SPEED = 7;

    private final double STRING_GEAR_RATIO = 4;
    private final double STRING_PULL_SPEED = 3;

    public ClimberSubsystem() {
        this.telescopeSpark = new SciSpark(PortMap.TELESCOPING_SPARK, TELESCOPING_GEAR_RATIO);
        this.stringTalon1   = new SciTalon(PortMap.STRING_TALON_1,    STRING_GEAR_RATIO);
        this.stringTalon2   = new SciTalon(PortMap.STRING_TALON_2,    STRING_GEAR_RATIO); 
        this.shiftMotor     = new SciSpark(PortMap.SHIFTING_TALON,    CASCADE_GEAR_RATIO);

    }

    public void setShiftMotorSpeed(double speed) {this.shiftMotor.set(speed);}

    public void setTelescopingSpeed(double speed){ 
        this.telescopeSpark.set(speed);
    }
    public void moveTelescopeUp(){
        this.setTelescopingSpeed(TELESCOPING_UP_SPEED);
    }
    public void moveTelescopeDown(){
        this.setTelescopingSpeed(TELESCOPING_DOWN_SPEED);
    }

    public void setStringPullSpeed(double speed){
        this.stringTalon1.set(speed);
        this.setTelescopingSpeed(TELESCOPING_UP_SPEED);
    }
    public void pullString(){
        this.setStringPullSpeed(STRING_PULL_SPEED);
    }

    public void updateRobotState(){
        Robot.set(SD.ShiftSparkAngle, this.shiftMotor.getWheelAngle());
    }
    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }

}