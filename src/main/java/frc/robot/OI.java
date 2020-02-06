
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.sciSensorsActuators.SciJoystick;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton circleControllerButton, pointOneButton, pointTwoButton, pointThreeButton;

    
    public OI() {
        leftStick = new SciJoystick(PortMap.JOYSTICK_LEFT);
        rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        circleControllerButton = new JoystickButton(leftStick, PortMap.JOYSTICK_TRIGGER);
        circleControllerButton.whileActiveContinuous(new CircleControllerCommand());

        pointOneButton = new JoystickButton(leftStick, PortMap.JOYSTICK_LEFT_BUTTON);
        pointOneButton.whenPressed(new PointOneCommand());

        pointTwoButton = new JoystickButton(leftStick, PortMap.JOYSTICK_RIGHT_BUTTON);
        pointTwoButton.whenPressed(new PointTwoCommand());

        
        pointThreeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_CENTER_BUTTON);
        pointThreeButton.whenPressed(new PointThreeCommand());       
    }
}

