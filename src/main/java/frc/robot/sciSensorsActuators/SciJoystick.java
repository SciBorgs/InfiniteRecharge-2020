package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Joystick;

public class SciJoystick extends Joystick {

    private double inputDeadZone = 0.11; //deadzone because the joysticks are bad and they detect input when there is none

    public SciJoystick(int port) {super(port);}
    public SciJoystick(int port, double inputDeadZone) {        
        this(port);
        this.inputDeadZone = inputDeadZone;
    }

    public void setInputDeadZone(double inputDeadZone) {this.inputDeadZone = inputDeadZone; }
    public double getInputDeadZone() {return this.inputDeadZone;}

    public double deadzone(double output) {
        return Math.abs(output) < this.inputDeadZone ? 0 : output;
    }

    //Cannot override final method 
    public double getProcessedX() {return  this.deadzone(this.getX());}
    public double getProcessedY() {return -this.deadzone(this.getY());}    
}