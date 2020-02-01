package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    private SciSpark shiftSpark;
    private SciTalon telescopeTalon1, telescopeTalon2;
    private SciTalon stringTalon1, stringTalon2;

    private final double SHIFT_GEAR_RATIO = 1; // 

    private final double TELESCOPING_GEAR_RATIO = 90;
    private final double TELESCOPING_UP_SPEED   = 1;
    private final double TELESCOPING_DOWN_SPEED = -1;

    private final double STRING_GEAR_RATIO = 90;
    private final double STRING_PULL_SPEED = 1;
    private final double STRING_PUSH_SPEED = -1;

    public ClimberSubsystem() {
        this.telescopeTalon2 = new SciTalon(PortMap.TELESCOPE_TALON_1, TELESCOPING_GEAR_RATIO); 
        this.telescopeTalon2 = new SciTalon(PortMap.TELESCOPE_TALON_2, TELESCOPING_GEAR_RATIO);
        this.stringTalon1    = new SciTalon(PortMap.STRING_TALON_1,    STRING_GEAR_RATIO);
        this.stringTalon2    = new SciTalon(PortMap.STRING_TALON_2,    STRING_GEAR_RATIO);

        this.shiftSpark      = new SciSpark(PortMap.SHIFT_SPARK, SHIFT_GEAR_RATIO);

        this.telescopeTalon2.follow(this.telescopeTalon1);
        this.stringTalon2.follow(this.stringTalon1);
    }

   // public void setShiftMotorSpeed(double speed) {this.shiftMotor.set(speed);}

    public void setTelescopingSpeed(double speed) {this.telescopeTalon1.set(speed, 2);}
    public void moveTelescopeUp()                 {this.setTelescopingSpeed(TELESCOPING_UP_SPEED);  }
    public void moveTelescopeDown()               {this.setTelescopingSpeed(TELESCOPING_DOWN_SPEED);}
    public void stopTelescope()                   {this.setTelescopingSpeed(0);}

    public void setStringPullSpeed(double speed) {this.stringTalon1.set(speed, 2);}
    public void pullString()                     {this.setStringPullSpeed(STRING_PULL_SPEED);}
    public void reverseString()                  {this.setStringPullSpeed(STRING_PUSH_SPEED);}
    public void stopPullingString()              {this.setStringPullSpeed(0);}

    public void setShiftMotorSpeed(double speed) {this.shiftSpark.set(speed, 2);}

    public void updateRobotState(){
        Robot.set(SD.ShiftSparkAngle, this.shiftSpark.getWheelAngle());
    }

    
    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }

}