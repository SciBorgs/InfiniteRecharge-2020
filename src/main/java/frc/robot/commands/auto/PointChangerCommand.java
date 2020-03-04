package frc.robot.commands.auto;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PointChangerCommand extends InstantCommand {
    private int index = 0;
    @Override
    protected void execute() {
        index = (index + 1) % Robot.path.size();
        Robot.CURRENT_DESTINATION = Robot.path.get(index);
    }
}