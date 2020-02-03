package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.BallTrajectoryController;

public class ShootCommand extends Command {
  private boolean foundOptimalParameters;

  public ShootCommand() {
    requires(Robot.shooterSubsystem);
  }

  @Override
  protected void execute() {
    if (BallTrajectoryController.areParametersOptimal()) {
      BallTrajectoryController.setHoodAngle();
      BallTrajectoryController.shoot();
      this.foundOptimalParameters = true;
    } else {
      BallTrajectoryController.optimizeParameters();
    }
  }

  @Override
  protected boolean isFinished() {
    return this.foundOptimalParameters;
  }
}
