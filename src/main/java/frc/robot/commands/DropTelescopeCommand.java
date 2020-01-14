package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class DropTelescopeCommand extends InstantCommand {
    public DropTelescopeCommand() {}

    @Override
    protected void execute() {
        Robot.climberSubsystem.moveTelescopeDown();
    }
}