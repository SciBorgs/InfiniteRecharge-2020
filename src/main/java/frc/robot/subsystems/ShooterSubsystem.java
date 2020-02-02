package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;

public class ShooterSubsystem extends Subsystem {
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
  }

  public void setHoodAngle() {
    this.hoodAnglePID.addMeasurement(
        Robot.get(SD.HoodSparkWheelAngle) - this.absEncoder.getRadians());
    this.hoodSpark.set(this.hoodAnglePID.getOutput());
  }

  public void shoot() {
    this.shooterSparkVelocityPID.setReference(
        Utils.RPSToRPM(Robot.get(SD.ShooterSparkRPS)), ControlType.kVelocity);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
  }
}
