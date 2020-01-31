package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Joystick;

public class SciJoystick extends Joystick {

    private double inputDeadZoneX = 0.11; //deadzone because the joysticks are bad and they detect input when there is none
    private double inputDeadZoneY = 0.11; //deadzone because the joysticks are bad and they detect input when there is none

    public SciJoystick(int port) {super(port);}

    public void setInputDeadZoneX(double inputDeadZoneX) {this.inputDeadZoneX = inputDeadZoneX; }
    public void setInputDeadZoneY(double inputDeadZoneY) {this.inputDeadZoneY = inputDeadZoneY; }

    public double getInputDeadZoneX() {return this.inputDeadZoneX;}
    public double getInputDeadZoneY() {return this.inputDeadZoneY;}

    public double deadzoneX(double output) { return Math.abs(output) < this.inputDeadZoneX ? 0 : output; }
    public double deadzoneY(double output) { return Math.abs(output) < this.inputDeadZoneY ? 0 : output; }
    
    //Cannot override final method 
    public double getProcessedX() {return this.deadzone(this.getX());}
    public double getProcessedY() {return -this.deadzone(this.getY());}    
}
