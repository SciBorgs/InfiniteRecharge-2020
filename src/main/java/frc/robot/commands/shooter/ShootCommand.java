package frc.robot.commands.shooter;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonProcessingException;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.PowerCellTrajectoryController;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState.SD;

public class ShootCommand extends Command {
  private PowerCellTrajectoryController powerCellTrajectoryController;

  public ShootCommand() throws JsonProcessingException, IOException {
    this.powerCellTrajectoryController = new PowerCellTrajectoryController();
  }

  @Override
  protected void execute() {
    Pair<Double, Double> optimalParameters = this.powerCellTrajectoryController.getOptimalParameters();
    Robot.shooterSubsystem.setHoodAngle(optimalParameters.first);
    Robot.shooterSubsystem.setShooterOmega(optimalParameters.second);
    System.out.println("RPS Difference: " + (optimalParameters.second - Robot.get(SD.ShooterOmega)));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
