package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.controllers.BallTrajectoryController;
import frc.robot.helpers.DelayedPrinter;

public class ShootCommand extends Command {

  public ShootCommand() {
    requires(Robot.shooterSubsystem);
  }

  @Override 
  protected void initialize(){
    while (!BallTrajectoryController.areParametersOptimal()){
      BallTrajectoryController.optimizeParameters();
    }
  }

  @Override
  protected void execute() {
    if (BallTrajectoryController.areParametersOptimal()) {
      // System.out.println("ANGLE DIFF: " + Math.abs(BallTrajectoryController.getHoodAngle() - Robot.get(SD.HoodAngle)));
      System.out.println("RPS DIFF: " + (Robot.shooterSubsystem.RPMToOmega(BallTrajectoryController.getMotorRPM()) - Robot.shooterSubsystem.shooterSparkEncoder.getVelocity()));
      BallTrajectoryController.setHoodAngle();
      BallTrajectoryController.shoot();
    }
  }

  @Override
  protected boolean isFinished(){
    return !Robot.oi.centerButton.get();
  }

  @Override 
  protected void end(){
    Robot.shooterSubsystem.testHoodSpark(0);
    Robot.shooterSubsystem.setShooterSpark(0);
  }
}
