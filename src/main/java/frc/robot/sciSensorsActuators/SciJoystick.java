package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Joystick;

public class SciJoystick extends Joystick {

    public SciJoystick(int port) {
        super(port);
    }

    private static final double INPUT_DEADZONE = 0.11; // deadzone because the joysticks are bad and they detect input when there is none

    public double deadzone(double output) { //from driver subsystem
        return Math.abs(output) < INPUT_DEADZONE ? 0 : output;
    }

    //Cannot override final method 
    public double getProcessedY() {
        return -deadzone(this.getY());
    }
}
