package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.sciSensorsActuators.SciJoystick;/*
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;*/
import frc.robot.commands.drive.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.turret.*;

// FILE HAS NOT BEEN CLEANED UP //
public class OI {
    public SciJoystick leftStick, rightStick;
    public XboxController xboxController;

    public JoystickButton circleControllerButton, pointChangerButton, intakeButton, shootButton, gearShiftButton, toggleDriveDirectionButton;

    public OI() {
        this.leftStick  = new SciJoystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new SciJoystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        this.intakeButton = new JoystickButton(this.rightStick, PortMap.JOYSTICK_CENTER_BUTTON);
        this.intakeButton.whenPressed(new IntakeSuckCommand());
        this.intakeButton.whenReleased(new IntakeStopCommand());

        this.gearShiftButton = new JoystickButton(this.leftStick, PortMap.JOYSTICK_RIGHT_BUTTON);
        this.gearShiftButton.whenPressed(new LowGearCommand());
        this.gearShiftButton.whenReleased(new HighGearCommand());

        this.toggleDriveDirectionButton = new JoystickButton(this.rightStick, PortMap.JOYSTICK_TRIGGER);
        this.toggleDriveDirectionButton.whenPressed(new ToggleDriveDirection());

        this.shootButton = new JoystickButton(this.rightStick, PortMap.JOYSTICK_TRIGGER);
        this.shootButton.whenActive(new ShootCommand());
        this.shootButton.whenReleased(new StopShooterCommand());
    }
}

