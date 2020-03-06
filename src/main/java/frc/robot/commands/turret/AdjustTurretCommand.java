package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AdjustTurretCommand extends Command {

  @Override
  protected void execute() {
    Robot.turretSubsystem.rotateToAngle(
        Math.toRadians(
            Robot.limelightSubsystem.getTableData(
                Robot.limelightSubsystem.getCameraTable(), "tx")));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}