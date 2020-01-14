package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public JoystickButton toggleClawButton, adjustTiltButton, raiseTelescopeButton, pullStringButton;
    public XboxController xboxController;
    
    public OI() {
        leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        adjustTiltButton     = new JoystickButton(rightStick, PortMap.JOYSTICK_TRIGGER);
        raiseTelescopeButton = new JoystickButton(rightStick, PortMap.JOYSTICK_TRIGGER); // change
        pullStringButton     = new JoystickButton(rightStick, PortMap.JOYSTICK_TRIGGER); // change

        adjustTiltButton.whileHeld(new JoystickShiftCommand());
        raiseTelescopeButton.whenPressed(new RaiseTelescopeCommand());
        pullStringButton.whenPressed(new OnRungCommand());
    }
}
