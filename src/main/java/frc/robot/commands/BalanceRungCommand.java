package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Command;

public class BalanceRungCommand extends Command {
    private final String FILENAME = "BalanceRungCommand";

    public BalanceRungCommand() {}

    @Override
    protected void execute() {
        Robot.climberController.moveToBalance();
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.get(SD.ShiftSparkAngle)) < Math.toRadians(3);
    }
}