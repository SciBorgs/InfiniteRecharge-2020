package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ReverseStringCommand extends InstantCommand {
    public ReverseStringCommand() {}
    
    @Override
    protected void execute() {
        Robot.climberSubsystem.reverseString();
    }
}