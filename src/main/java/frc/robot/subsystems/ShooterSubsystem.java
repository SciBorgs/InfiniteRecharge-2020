package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.logging.*;
import frc.robot.logging.Logger.DefaultValue;

public class ShooterSubsystem extends Subsystem {
  private SciSpark spark;
  private DoubleSolenoid hoodSolenoid;
  private final double WHEEL_RADIUS = Utils.inchesToMeters(3);
  private final double INCREMENT = 0.05;
  private final double GEAR_RATIO = 1;

  public ShooterSubsystem() {
    this.spark = new SciSpark(13, GEAR_RATIO);
    //this.hoodSolenoid = Utils.newDoubleSolenoid(new int[]{-1,-1});
  }

  public void incrementSpeed(){this.spark.updateSpeed(INCREMENT);}
  public void decrementSpeed(){this.spark.updateSpeed(-INCREMENT);}

  public void setSpeed(double speed) {
    this.spark.set(speed);
  }

  public void logSpeed() {
    double rpm = this.spark.getEncoder().getVelocity();
    System.out.println("RPM: " + rpm);
    System.out.println("Speed: " + this.spark.get());
    Robot.logger.addData("SHOOTER SUBSYSTEM", "Radians/min",rpm, DefaultValue.Previous);
  }

  public void toggleHood() {
    Utils.toggleDoubleSolenoid(hoodSolenoid);
  }


  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}