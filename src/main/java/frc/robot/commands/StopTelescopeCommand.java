package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopTelescopeCommand extends InstantCommand {
    public StopTelescopeCommand() {}

    @Override
    protected void execute() {
        Robot.climberSubsystem.stopTelescope();
    }
}