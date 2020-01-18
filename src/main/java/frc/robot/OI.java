package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.sciSensorsActuators.SciJoystick;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;

    public JoystickButton topInc, topDec, bottomInc, bottomDec;

    public XboxController xboxController;
    
    public OI() {
        leftStick = new SciJoystick(PortMap.JOYSTICK_LEFT);
        rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);
    }
}
