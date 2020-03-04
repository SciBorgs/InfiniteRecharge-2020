package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.sciSensorsActuators.SciJoystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.turret.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.intake.IntakeSuckCommand;
import frc.robot.commands.shooter.*;
import frc.robot.commands.turret.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton circleControllerButton, pointChangerButton, intakeButton;
    
    public OI() {
        this.leftStick  = new SciJoystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        this.intakeButton = new JoystickButton(this.rightStick, PortMap.JOYSTICK_CENTER_BUTTON);
        this.intakeButton.whenPressed(new IntakeSuckCommand());
        this.intakeButton.whenReleased(new IntakeStopCommand());


    }
}

