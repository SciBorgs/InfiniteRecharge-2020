package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem.GearRatio;

public class LowGearCommand extends InstantCommand {
    @Override
    protected void execute() {
        Robot.driveSubsystem.shiftSolenoid.set(GearRatio.Low);
    }
}