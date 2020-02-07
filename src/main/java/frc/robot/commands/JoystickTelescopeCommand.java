package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class JoystickTelescopeCommand extends InstantCommand {
    private final String FILENAME = "JoystickTelescopeCommand.java";

    public JoystickTelescopeCommand() {}

    @Override
    protected void execute() {
        Robot.climberSubsystem.setTelescopingSpeed(Robot.driveSubsystem.processStick(Robot.oi.leftStick));
    }
}