package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controllers.ClimberController;
import frc.robot.robotState.SD;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class BalanceRungCommand extends Command {
    private final String FILENAME = "BalanceRungCommand";

    public BalanceRungCommand() {}

    @Override
    protected void execute() {
        ClimberController.moveToBalance();
    }

    @Override
    protected void isFinished() {
        return Math.abs(Robot.get(ShiftSparkAngle)) < 8;
    }
}