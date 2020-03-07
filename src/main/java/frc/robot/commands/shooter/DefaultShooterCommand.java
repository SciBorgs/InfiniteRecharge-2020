package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class DefaultShooterCommand extends InstantCommand {

    @Override
    protected void execute(){
        Robot.shooterSubsystem.testShooterSpark(0.1);
        Robot.hopperSubsystem.elevator();
    }
}