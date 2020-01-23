package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public JoystickButton toggleClawButton, adjustTiltButton, raiseTelescopeButton, lowerTelescopeButton, pullStringButton;
    public XboxController xboxController;
    
    public OI() {
        leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        adjustTiltButton     = new JoystickButton(rightStick, PortMap.JOYSTICK_TRIGGER);
        adjustTiltButton.whileHeld(new JoystickShiftCommand());

        raiseTelescopeButton = new JoystickButton(rightStick, PortMap.JOYSTICK_CENTER_BUTTON); // change
        raiseTelescopeButton.whenPressed(new RaiseTelescopeCommand());
        raiseTelescopeButton.whenReleased(new StopTelescopeCommand());

        lowerTelescopeButton = new JoystickButton(rightStick, PortMap.JOYSTICK_TRIGGER);
        lowerTelescopeButton.whenPressed(new DropTelescopeCommand());
        lowerTelescopeButton.whenReleased(new StopTelescopeCommand());

        pullStringButton     = new JoystickButton(rightStick, PortMap.JOYSTICK_RIGHT_BUTTON); // change
        pullStringButton.whenPressed(new OnRungCommand());
    }
}
