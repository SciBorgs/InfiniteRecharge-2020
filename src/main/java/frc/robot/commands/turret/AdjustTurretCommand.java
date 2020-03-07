package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AdjustTurretCommand extends Command {
  private int ticksSinceLastFound = 0;
  private final int MAX_TICKS_SINCE_LAST_FOUND = 10;

  @Override
  protected void execute() {
    if (!Robot.limelightSubsystem.contourExists()) {
      this.ticksSinceLastFound++;
      Robot.turretSubsystem.setTurretSpeed(0);
    }
    else {
      this.ticksSinceLastFound = 0;
      Robot.turretSubsystem.pointTowardsTarget(
        Math.toRadians(
            Robot.limelightSubsystem.getTableData(
                Robot.limelightSubsystem.getCameraTable(), "tx")));
    }
    if (ticksSinceLastFound >= MAX_TICKS_SINCE_LAST_FOUND) {
      Robot.turretSubsystem.moveToInitialPosition();
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}