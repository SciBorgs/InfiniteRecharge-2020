package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopStringCommand extends InstantCommand {
    public StopStringCommand() {}
    
    @Override
    protected void execute() {
        Robot.climberSubsystem.stopPullingString();
    }
}