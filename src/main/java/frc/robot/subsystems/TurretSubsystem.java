package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
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

  private final double MAX_TURRET_ANGLE = Math.PI / 2;
  private final double MIN_TURRET_ANGLE = -MAX_TURRET_ANGLE;

  private final double TURRET_INITAL_ANGLE = 0;

  public TurretSubsystem() {
    this.turretEncoder = new SciThroughBoreEncoder(PortMap.TURRET_ENCODER);
    this.turretEncoder.setDistancePerRotation(2 * Math.PI * TURRET_SPARK_GEAR_RATIO);
    this.turretEncoder.setAngle(Math.toRadians(TURRET_INITAL_ANGLE));
    Robot.set(SD.HoodAngle, this.turretEncoder.getRadians());

    this.turretSpark = new SciSpark(PortMap.TURRET_SPARK);
    this.turretPID = new PID(TURRET_P, TURRET_I, TURRET_D);

    this.turretEncoder.assignSD(SciThroughBoreEncoderSD.Radians, SD.TurretAngle);
  }

  public void setTurretSpeed(double speed) {
    if ((Robot.get(SD.TurretAngle) >= MAX_TURRET_ANGLE && speed > 0) || 
        (Robot.get(SD.TurretAngle) <= MIN_TURRET_ANGLE && speed < 0)) {speed = 0;}
    this.turretSpark.set(speed);
  }

  public void pointTowardsTarget(double angle) {
    this.turretPID.addMeasurement(-angle);
    setTurretSpeed(this.turretPID.getOutput());
  }

  @Override
  protected void initDefaultCommand() {}
}