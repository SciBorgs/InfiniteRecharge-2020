package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class RaiseTelescopeCommand extends InstantCommand {
    public RaiseTelescopeCommand() {}

    @Override
    protected void execute() {
        Robot.climberSubsystem.moveTelescopeUp();
    }
}