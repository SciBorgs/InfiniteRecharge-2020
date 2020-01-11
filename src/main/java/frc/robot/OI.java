package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sciSensorsActuators.SciJoystick;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;
    public XboxController xboxController;
    
    public OI() {
        leftStick = new SciJoystick(PortMap.JOYSTICK_LEFT);
        rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);
    }
}
