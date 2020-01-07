package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;
    public JoystickButton intakeButton;
    
    public OI() {
        this.leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        this.intakeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_LEFT_BUTTON); // Temporary
        this.intakeButton.whenPressed(new IntakeSuckCommand());
        this.intakeButton.whenReleased(new IntakeReleaseCommand());
    }
}

