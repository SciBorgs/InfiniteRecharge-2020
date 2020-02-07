package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public JoystickButton toggleClawButton, adjustTiltButton, raiseTelescopeButton, lowerTelescopeButton, joystickTelescopeButton;
    public XboxController xboxController;
    
    public OI() {
        this.leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        //adjustTiltButton     = new JoystickButton(rightStick, PortMap.JOYSTICK_LEFT_BUTTON);
        //adjustTiltButton.whileHeld(new JoystickShiftCommand());

        raiseTelescopeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_LEFT_BUTTON); // change
        raiseTelescopeButton.whenPressed(new RaiseTelescopeCommand());
        raiseTelescopeButton.whenReleased(new StopTelescopeCommand());

        lowerTelescopeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_RIGHT_BUTTON);
        lowerTelescopeButton.whenPressed(new DropTelescopeCommand());
        lowerTelescopeButton.whenReleased(new StopTelescopeCommand());

        joystickTelescopeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_TRIGGER);
        joystickTelescopeButton.whileHeld(new JoystickTelescopeCommand());
        joystickTelescopeButton.whenReleased(new StopTelescopeCommand());
    }
}

