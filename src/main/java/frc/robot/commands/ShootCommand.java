package frc.robot.commands;

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
    Pair<Double, Double> optimalParamters = this.powerCellTrajectoryController.getOptimalParameters();
    Robot.shooterSubsystem.setHoodAngle(optimalParamters.first);
    Robot.shooterSubsystem.setShooterOmega(optimalParamters.second);
    System.out.println("RPS Difference: " + (optimalParamters.second - Robot.get(SD.ShooterOmega)));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
