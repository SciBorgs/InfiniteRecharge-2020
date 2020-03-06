package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder.SciThroughBoreEncoderSD;

public class TurretSubsystem extends Subsystem {
  private SciThroughBoreEncoder absEncoder;
  private SciSpark turretSpark;
  private PID turretPID;

  private final double TURRET_P = 0, TURRET_I = 0, TURRET_D = 0;
  private final double TURRET_SPARK_GEAR_RATIO = 1;

  public TurretSubsystem() {
    this.absEncoder = new SciThroughBoreEncoder(-1);
    this.absEncoder.setDistancePerRotation(2 * Math.PI * TURRET_SPARK_GEAR_RATIO);
    Robot.set(SD.HoodAngle, this.absEncoder.getRadians());

    this.turretSpark = new SciSpark(-1);
    this.turretPID = new PID(TURRET_P, TURRET_I, TURRET_D);

    this.absEncoder.assignSD(SciThroughBoreEncoderSD.Radians, SD.TurretAngle);
  }

  public void rotateToAngle(double angle) {
    this.turretPID.addMeasurement(angle - Robot.get(SD.TurretAngle));
    this.turretSpark.set(this.turretPID.getOutput());
  }

  @Override
  protected void initDefaultCommand() {}
}