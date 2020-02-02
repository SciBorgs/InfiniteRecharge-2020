package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.TrajectoryController;

public class ShootCommand extends Command {
  private boolean foundOptimalParameters;

  public ShootCommand() {
    requires(Robot.shooterSubsystem);
  }

  @Override
  protected void execute() {
    if (TrajectoryController.areParametersOptimal()) {
      Robot.shooterSubsystem.setHoodAngle();
      Robot.shooterSubsystem.shoot();
      this.foundOptimalParameters = true;
    } else {
      TrajectoryController.optimizeParameters();
    }
  }

  @Override
  protected boolean isFinished() {
    return this.foundOptimalParameters;
  }
}
