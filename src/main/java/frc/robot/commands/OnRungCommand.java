package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OnRungCommand extends CommandGroup {
    public OnRungCommand() {
        addSequential(new DropTelescopeCommand());
        addSequential(new PullStringCommand());
        addSequential(new BalanceRungCommand());
    }
}