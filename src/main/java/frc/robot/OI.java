package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton circleControllerButton;
    
    public OI() {
        leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        circleControllerButton = new JoystickButton(leftStick, PortMap.JOYSTICK_TRIGGER);
        circleControllerButton.whileHeld(new CircleControllerCommand());
        
    }
}
