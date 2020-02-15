package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;

public class ShooterSubsystem extends Subsystem implements RobotStateUpdater {
  public double HOOD_ANGLE_P = 0.65, HOOD_ANGLE_I = 0.01, HOOD_ANGLE_D = 0.0;
  public SciThroughBoreEncoder absEncoder;
  public CANEncoder shooterSparkEncoder;
  
  private final double SHOOTER_VELOCITY_P = 0.0009,
      SHOOTER_VELOCITY_I = 0.0000009,
      SHOOTER_VELOCITY_D = 0.00012;
  private final double HOOD_SPARK_GEAR_RATIO = 36.0 / 334.0;
  private final double SHOOTER_SPARK_GEAR_RATIO = 1;

  private final double SHOOTER_SPARK_MIN_OUTPUT = -1;
  private final double SHOOTER_SPARK_MAX_OUTPUT = 1;

  public SciSpark hoodSpark, rightShooterSpark, leftShooterSpark;

  private CANPIDController shooterSparkVelocityPID;
  private PID hoodAnglePID;

  public ShooterSubsystem() {
    this.hoodSpark = new SciSpark(1, HOOD_SPARK_GEAR_RATIO);
    this.rightShooterSpark = new SciSpark(7, SHOOTER_SPARK_GEAR_RATIO);
    this.rightShooterSpark.setInverted(true);
    this.leftShooterSpark  = new SciSpark(2, SHOOTER_SPARK_GEAR_RATIO);
    //this.leftShooterSpark.follow(rightShooterSpark, false);

    this.hoodAnglePID = new PID(HOOD_ANGLE_P, HOOD_ANGLE_I, HOOD_ANGLE_D);
    this.shooterSparkVelocityPID = this.rightShooterSpark.getPIDController();
    this.shooterSparkVelocityPID.setP(SHOOTER_VELOCITY_P);
    this.shooterSparkVelocityPID.setI(SHOOTER_VELOCITY_I);
    this.shooterSparkVelocityPID.setD(SHOOTER_VELOCITY_D);
    this.shooterSparkVelocityPID.setOutputRange(SHOOTER_SPARK_MIN_OUTPUT, SHOOTER_SPARK_MAX_OUTPUT);

    this.absEncoder = new SciThroughBoreEncoder(1);
    this.absEncoder.setDistancePerRotation(2 * Math.PI * HOOD_SPARK_GEAR_RATIO);
    this.shooterSparkEncoder = this.rightShooterSpark.getEncoder();
    Robot.set(SD.HoodAngle, this.absEncoder.getRadians());
    Robot.addRobotStateUpdater(this);
  }

  public void setHoodSpark(double angle) {
//    System.out.println("GOING TO: " + angle);
    System.out.println("AT: " + Robot.get(SD.HoodAngle));
    this.hoodAnglePID.addMeasurement(angle - Robot.get(SD.HoodAngle));
    this.hoodSpark.set(this.hoodAnglePID.getOutput());
  }

  public void testHoodSpark(double speed) {
    this.hoodSpark.set(speed);
  }

  public void setShooterSpark(double RPM) {
    this.shooterSparkVelocityPID.setReference(RPM, ControlType.kVelocity);
  }

  public void testShooterSpark2(double speed) {
    this.rightShooterSpark.set(speed);
  }

  public void testShooterSpark7(double speed) {
    this.leftShooterSpark.set(speed);
  }

  public double RPMToOmega(double RPM) {
    return RPM * Math.PI / 30;
  }

  @Override
  protected void initDefaultCommand() {}

  @Override
  public void updateRobotState() {
    Robot.set(SD.HoodAngle, this.absEncoder.getRadians());
    Robot.set(SD.ShooterSparkOmega, RPMToOmega(this.shooterSparkEncoder.getVelocity()));
  }
}
