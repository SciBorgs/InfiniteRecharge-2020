package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;

public class ShooterSubsystem extends Subsystem implements RobotStateUpdater {
  private final double HOOD_ANGLE_P = 0, HOOD_ANGLE_I = 0, HOOD_ANGLE_D = 0;
  private final double SHOOTER_VELOCITY_P = 0.001,
      SHOOTER_VELOCITY_I = 0.000001,
      SHOOTER_VELOCITY_D = 0.01;
  private final double HOOD_SPARK_GEAR_RATIO = 36.0 / 3449;
  private final double SHOOTER_SPARK_GEAR_RATIO = 1;

  private final double SHOOTER_SPARK_MIN_OUTPUT = -1;
  private final double SHOOTER_SPARK_MAX_OUTPUT = 1;

  private SciSpark hoodSpark, shooterSpark;
  private SciThroughBoreEncoder absEncoder;

  private CANPIDController shooterSparkVelocityPID;
  private CANEncoder shooterSparkEncoder;
  private PID hoodAnglePID;

  public ShooterSubsystem() {
    this.hoodSpark = new SciSpark(PortMap.HOOD_SPARK, HOOD_SPARK_GEAR_RATIO);
    this.shooterSpark = new SciSpark(PortMap.SHOOTER_SPARK, SHOOTER_SPARK_GEAR_RATIO);

    this.hoodAnglePID = new PID(HOOD_ANGLE_P, HOOD_ANGLE_I, HOOD_ANGLE_D);
    this.shooterSparkVelocityPID = this.shooterSpark.getPIDController();
    this.shooterSparkVelocityPID.setP(SHOOTER_VELOCITY_P);
    this.shooterSparkVelocityPID.setI(SHOOTER_VELOCITY_I);
    this.shooterSparkVelocityPID.setD(SHOOTER_VELOCITY_D);
    this.shooterSparkVelocityPID.setOutputRange(SHOOTER_SPARK_MIN_OUTPUT, SHOOTER_SPARK_MAX_OUTPUT);

    this.absEncoder = new SciThroughBoreEncoder(PortMap.THROUGH_BORE_ENCODER);
    this.shooterSparkEncoder = this.shooterSpark.getEncoder();
  }

  public void setHoodSpark(double angle) {
    this.hoodAnglePID.addMeasurement(
        angle - Robot.get(SD.HoodAngle));
    this.hoodSpark.set(this.hoodAnglePID.getOutput());
  }

  // TODO: Remove this method
  public void testHoodSpark(double speed) {
    this.hoodSpark.set(speed);
  }

  public void setShooterSpark(double RPM) {
    this.shooterSparkVelocityPID.setReference(RPM, ControlType.kVelocity);
  }

  private double RPMToOmega(double RPM) {
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
