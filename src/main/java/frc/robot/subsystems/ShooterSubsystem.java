package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utils;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.sciSensorsActuators.SciSpark;

public class ShooterSubsystem extends Subsystem {
  private SciSpark spark;
  private final double WHEEL_RADIUS = Utils.inchesToMeters(3);

  public ShooterSubsystem() {
    this.spark = new SciSpark(-1);
  }

  public void setSpeed(double speed) {
    this.spark.set(speed);
  }

  public void logSpeed() {
    double rpm = this.spark.getEncoder().getVelocity();
    DelayedPrinter.print("RPM: " + rpm + "\t m/s: " + WHEEL_RADIUS * 2 * Math.PI * rpm / 60);
  }

  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}