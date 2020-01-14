package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class JoystickShiftCommand extends InstantCommand {
    private final String FILENAME = "JoystickShiftCommand.java";

    public JoystickShiftCommand() {}
    
    @Override
    protected void execute() {
        Robot.climberSubsystem.setShiftMotorSpeed(Robot.driveSubsystem.processStick(Robot.oi.rightStick));
    }
}