package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ShooterCommand;
import frc.robot.sciSensorsActuators.SciJoystick;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;

    public JoystickButton topInc, topDec, bottomInc, bottomDec;

    public XboxController xboxController;
    
    public OI() {
        leftStick = new SciJoystick(PortMap.JOYSTICK_LEFT);
        rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        topInc = new JoystickButton(leftStick, 1);
        topDec = new JoystickButton(leftStick, 2);
        bottomInc = new JoystickButton(leftStick, 3);
        bottomDec = new JoystickButton(leftStick, 4);

        topInc.whenPressed(new ShooterCommand(Robot.shooterSubsystem.topOutput + 0.05, Robot.shooterSubsystem.bottomOutput));
        topDec.whenPressed(new ShooterCommand(Robot.shooterSubsystem.topOutput - 0.05, Robot.shooterSubsystem.bottomOutput));

        bottomInc.whenPressed(new ShooterCommand(Robot.shooterSubsystem.topOutput, Robot.shooterSubsystem.bottomOutput + 0.05));
        bottomDec.whenPressed(new ShooterCommand(Robot.shooterSubsystem.topOutput, Robot.shooterSubsystem.bottomOutput - 0.05));
    }
}
