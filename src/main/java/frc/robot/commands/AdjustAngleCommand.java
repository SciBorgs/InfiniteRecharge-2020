package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Command;

public class AdjustAngleCommand extends Command {
    private final double targetAngle = Math.toRadians(-62.5);
    private final String FILENAME = "AdjustAngleCommand.java";

    public AdjustAngleCommand() {}

    protected void execute() {
        Robot.angleController.goToAngle(-62.5);
    }

    protected boolean isFinished() {
        return Math.abs(Robot.get(SD.Angle) - targetAngle) < Math.toRadians(1);
    }
}