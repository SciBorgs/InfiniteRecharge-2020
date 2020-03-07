package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.controllers.PowerCellTrajectoryController;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState.SD;

public class ShootCommand extends InstantCommand {
  private PowerCellTrajectoryController powerCellTrajectoryController;

  public ShootCommand() {
    this.powerCellTrajectoryController = new PowerCellTrajectoryController();
  }

  @Override
  protected void execute() {
    Pair<Double, Double> optimalParameters = this.powerCellTrajectoryController.getOptimalParameters();
    Robot.shooterSubsystem.setHoodAngle(optimalParameters.first);
    Robot.shooterSubsystem.setShooterOmega(optimalParameters.second);
    Robot.hopperSubsystem.elevator();
  }
}
