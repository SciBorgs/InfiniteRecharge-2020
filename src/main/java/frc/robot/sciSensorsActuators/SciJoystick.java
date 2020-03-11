package frc.robot.sciSensorsActuators;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciJoystick.SciJoystickSD;

public class SciJoystick extends Joystick implements RobotStateUpdater,SciSensorActuator<SciJoystickSD> {
    public static enum SciJoystickSD {Y};
    public HashMap<SciJoystickSD, SD> sdMap;

    private double inputDeadZoneX = 0.11; //deadzone because the joysticks are bad and they detect input when there is none
    private double inputDeadZoneY = 0.11; //deadzone because the joysticks are bad and they detect input when there is none

    public SciJoystick(int port) {
        super(port);
        this.sdMap = new HashMap<>();
    }

    public void setInputDeadZoneX(double inputDeadZoneX) {this.inputDeadZoneX = inputDeadZoneX; }
    public void setInputDeadZoneY(double inputDeadZoneY) {this.inputDeadZoneY = inputDeadZoneY; }

    public double getInputDeadZoneX() {return this.inputDeadZoneX;}
    public double getInputDeadZoneY() {return this.inputDeadZoneY;}

    public double deadzoneX(double output) { return Math.abs(output) < this.inputDeadZoneX ? 0 : output; }
    public double deadzoneY(double output) { return Math.abs(output) < this.inputDeadZoneY ? 0 : output; }
    
    //Cannot override final method 
    public double getProcessedX() {return  this.deadzoneX(this.getX());}
    public double getProcessedY() {return -this.deadzoneY(this.getY());}    

    @Override
    public HashMap<SciJoystickSD, SD> getSDMap() { return this.sdMap; }

    @Override
    public String getDeviceName() { return "Joystick on port: " + super.getPort(); }

    @Override
    public void updateRobotState() { sciSet(SciJoystickSD.Y, super.getY()); }
}