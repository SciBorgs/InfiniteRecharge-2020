
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.auto.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drive.*;
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
        circleControllerButton.whileActive(new CircleControllerCommand());
        
        pointThreeButton = new JoystickButton(leftStick, PortMap.JOYSTICK_CENTER_BUTTON);
        pointThreeButton.whenPressed(new PointThreeCommand());       
    }
}

