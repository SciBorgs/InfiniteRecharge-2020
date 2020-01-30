package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShootCommand extends Command {
    private boolean foundOptimalParameters;
    
    public ShootCommand() {
        requires(Robot.shooterSubsystem);
    }
    
    @Override
    protected void execute() {
        if (Robot.shooterSubsystem.areParametersOptimal()) {
            Robot.shooterSubsystem.setHoodAngle();
            Robot.shooterSubsystem.shoot();
            this.foundOptimalParameters = true;
        } else {
            Robot.shooterSubsystem.optimizeParameters();
        }
    }

    @Override
    protected boolean isFinished() {
        return foundOptimalParameters;
    }
}