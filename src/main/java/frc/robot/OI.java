
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.sciSensorsActuators.SciJoystick;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton circleControllerButton, pointOneButton, pointTwoButton, pointThreeButton;

    public JoystickButton centerButton;

    public OI() {
        this.leftStick = new SciJoystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);     

        this.centerButton = new JoystickButton(leftStick, PortMap.JOYSTICK_CENTER_BUTTON);
        this.centerButton.whenPressed(new ShootCommand());
        //this.centerButton.whenReleased(new StopShooterCommand());
        
    }
}

