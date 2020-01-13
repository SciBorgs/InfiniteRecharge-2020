package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OnRungCommand extends CommandGroup {
    public OnRungCommand() {
        addSequential(new PullUpCommand());
        addSequential(new BalanceRungCommand());
    }
}