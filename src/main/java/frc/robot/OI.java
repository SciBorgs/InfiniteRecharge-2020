package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton toggleGears, toggleOutput;

    public OI() {
        leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        toggleGears  = new JoystickButton(rightStick, PortMap.JOYSTICK_RIGHT_BUTTON);
        toggleGears.whenPressed(new ToggleGearsCommand());
        
        toggleOutput = new JoystickButton(leftStick , PortMap.JOYSTICK_LEFT_BUTTON);
        toggleGears.whenPressed(new ToggleOutputCommand());
    }
}
