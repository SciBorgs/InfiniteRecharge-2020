package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utils;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.sciSensorsActuators.SciSpark;

public class ShooterSubsystem extends Subsystem {
  private SciSpark spark;
  private DoubleSolenoid hoodSolenoid;
  private final double WHEEL_RADIUS = Utils.inchesToMeters(3);
  private final double INCREMENT = 0.5;

  public ShooterSubsystem() {
    this.spark = new SciSpark(3);
    //this.hoodSolenoid = Utils.newDoubleSolenoid(new int[]{-1,-1});
  }

  public void incrementSpeed(){this.spark.updateSpeed(INCREMENT);}
  public void decrementSpeed(){this.spark.updateSpeed(-INCREMENT);}

  public void setSpeed(double speed) {
    this.spark.set(speed);
  }

  public void logSpeed() {
    double rpm = this.spark.getEncoder().getVelocity();
    DelayedPrinter.print("RPM: " + rpm);
  }

  public void toggleHood() {
    Utils.toggleDoubleSolenoid(hoodSolenoid);
  }

  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}