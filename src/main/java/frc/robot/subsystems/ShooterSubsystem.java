package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

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
  private CANPIDController pidController;
  private DoubleSolenoid hoodSolenoid;
  private final double WHEEL_RADIUS = Utils.inchesToMeters(3);
  private final double INCREMENT = 0.01;
  private final double GEAR_RATIO = 1;

  public ShooterSubsystem() {
    this.spark = new SciSpark(33, GEAR_RATIO);
    this.pidController = this.spark.getPIDController();
    this.pidController.setP(0.001);
    this.pidController.setI(0.000001);
    this.pidController.setD(0.01);
    this.pidController.setOutputRange(-1, 1);

    //this.hoodSolenoid = Utils.newDoubleSolenoid(new int[]{-1,-1});
  }

  public void incrementSpeed(){this.spark.updateSpeed(INCREMENT);}
  public void decrementSpeed(){this.spark.updateSpeed(-INCREMENT);}

  public void setSpeed(double rpm) {
    //this.spark.set(rpm);
    this.pidController.setReference(rpm, ControlType.kVelocity);
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