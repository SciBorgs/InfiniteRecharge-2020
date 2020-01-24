package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;
    public JoystickButton intakeStartButton;
    public JoystickButton intakeStopButton;
    
    public OI() {
        this.leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        this.intakeStartButton = new JoystickButton(rightStick, PortMap.JOYSTICK_LEFT_BUTTON); // Temporary
        this.intakeStartButton.whenPressed(new IntakeSuckCommand());
        this.intakeStopButton = new JoystickButton(rightStick, PortMap.JOYSTICK_RIGHT_BUTTON);
        this.intakeStopButton.whenPressed(new IntakeReleaseCommand());
    }
}

