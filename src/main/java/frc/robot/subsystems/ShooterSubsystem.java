package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;
import frc.robot.sciSensorsActuators.SciSpark.SciSparkSD;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder.SciThroughBoreEncoderSD;

public class ShooterSubsystem extends Subsystem {
  private double HOOD_ANGLE_P = 0.65, HOOD_ANGLE_I = 0.01, HOOD_ANGLE_D = 0.0;
  private final double SHOOTER_VELOCITY_P = 0.0009,
      SHOOTER_VELOCITY_I = 0.0000009,
      SHOOTER_VELOCITY_D = 0.00012;

  private final double HOOD_INITIAL_ANGLE = 17;

  private final double HOOD_SPARK_GEAR_RATIO = 36.0 / 334.0;
  private final double SHOOTER_SPARK_GEAR_RATIO = 1;

  private final double SHOOTER_SPARK_MIN_OUTPUT = -1;
  private final double SHOOTER_SPARK_MAX_OUTPUT = 1;

  private SciSpark hoodSpark, rightShooterSpark, leftShooterSpark;
  private SciThroughBoreEncoder hoodEncoder;

  private CANPIDController shooterSparkVelocityPID;
  private PID hoodAnglePID;

  public ShooterSubsystem() {
    this.hoodSpark = new SciSpark(PortMap.HOOD_SPARK, HOOD_SPARK_GEAR_RATIO);
    this.rightShooterSpark = new SciSpark(PortMap.SHOOTER_RIGHT_SPARK, SHOOTER_SPARK_GEAR_RATIO);
    this.rightShooterSpark.setInverted(true);
    this.leftShooterSpark = new SciSpark(PortMap.SHOOTER_LEFT_SPARK, SHOOTER_SPARK_GEAR_RATIO);
    this.leftShooterSpark.follow(rightShooterSpark);

    this.hoodAnglePID = new PID(HOOD_ANGLE_P, HOOD_ANGLE_I, HOOD_ANGLE_D);
    this.shooterSparkVelocityPID = this.rightShooterSpark.getPIDController();
    this.shooterSparkVelocityPID.setP(SHOOTER_VELOCITY_P);
    this.shooterSparkVelocityPID.setI(SHOOTER_VELOCITY_I);
    this.shooterSparkVelocityPID.setD(SHOOTER_VELOCITY_D);
    this.shooterSparkVelocityPID.setOutputRange(SHOOTER_SPARK_MIN_OUTPUT, SHOOTER_SPARK_MAX_OUTPUT);

    /*
    this.hoodEncoder = new SciThroughBoreEncoder(PortMap.HOOD_ENCODER);
    this.hoodEncoder.setDistancePerRotation(2 * Math.PI * HOOD_SPARK_GEAR_RATIO);
    this.hoodEncoder.setAngle(Math.toRadians(HOOD_INITIAL_ANGLE));*/
    Robot.set(SD.HoodAngle, 0);// this.hoodEncoder.getRadians());
    
    this.rightShooterSpark.assignSD(SciSparkSD.Velocity, SD.ShooterOmega);
    //this.hoodEncoder.assignSD(SciThroughBoreEncoderSD.Radians, SD.HoodAngle);
  }

  public void setHoodAngle(double angle) {
    this.hoodAnglePID.addMeasurement(angle - Robot.get(SD.HoodAngle));
    this.hoodSpark.set(this.hoodAnglePID.getOutput());
  }

  public void setShooterOmega(double RPS) {
    this.shooterSparkVelocityPID.setReference(RPS, ControlType.kVelocity);
  }

  public void testShooterSpark(double speed){
    DelayedPrinter.print("setting to: " + speed);
    this.rightShooterSpark.set(speed);
    DelayedPrinter.print("is at: " + this.rightShooterSpark.get());
  }

  public void stopMotors() {
    this.hoodSpark.stopMotor();
    this.rightShooterSpark.stopMotor();
    this.leftShooterSpark.stopMotor();
  }

  @Override
  protected void initDefaultCommand() {}
}
