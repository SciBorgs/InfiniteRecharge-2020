package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder.SciThroughBoreEncoderSD;

public class TurretSubsystem extends Subsystem {
  private SciThroughBoreEncoder turretEncoder;
  private SciSpark turretSpark;
  private PID turretPID;

  private final double TURRET_P = 0, TURRET_I = 0, TURRET_D = 0;
  private final double TURRET_SPARK_GEAR_RATIO = 1;

  public TurretSubsystem() {
    this.turretEncoder = new SciThroughBoreEncoder(PortMap.TURRET_ENCODER);
    this.turretEncoder.setDistancePerRotation(2 * Math.PI * TURRET_SPARK_GEAR_RATIO);
    Robot.set(SD.HoodAngle, this.turretEncoder.getRadians());

    this.turretSpark = new SciSpark(PortMap.TURRET_SPARK);
    this.turretPID = new PID(TURRET_P, TURRET_I, TURRET_D);

    this.turretEncoder.assignSD(SciThroughBoreEncoderSD.Radians, SD.TurretAngle);
  }

  public void rotateToAngle(double angle) {
    this.turretPID.addMeasurement(angle - Robot.get(SD.TurretAngle));
    this.turretSpark.set(this.turretPID.getOutput());
  }

  @Override
  protected void initDefaultCommand() {}
}