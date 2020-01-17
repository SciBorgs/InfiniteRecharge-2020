package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ShooterCommand extends InstantCommand {
    public ShooterCommand(double topOutput, double bottomOutput) {
        requires(Robot.shooterSubsystem);
        Robot.shooterSubsystem.topOutput = topOutput;
        Robot.shooterSubsystem.bottomOutput = bottomOutput;
    }

    @Override 
    protected void execute() {
        Robot.shooterSubsystem.setSpeed();
    }
}