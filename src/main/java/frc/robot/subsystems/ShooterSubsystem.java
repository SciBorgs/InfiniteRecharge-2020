package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  private SciTalon rotateTalon;
  private SciSpark shooterMotor;
  
  private final int LEFT_EXTREME_ANGLE = -90;
  private final int RIGHT_EXTREME_ANGLE = 90;

  private DigitalInput limitSwitchLeft;  // -90 deg
  private DigitalInput limitSwitchRight; // 90 deg

  private PID shooterPID;
  private final int SHOOTER_P = 0;
  private final int SHOOTER_I = 0;
  private final int SHOOTER_D = 0;

  public ShooterSubsystem() {
    this.rotateTalon  = new SciTalon(PortMap.SHOOTER_ROTATE_SPARK);
    this.shooterMotor = new SciSpark(PortMap.SHOOTER_MOTOR);
    this.limitSwitchLeft  = new DigitalInput(PortMap.SHOOTER_LEFT_DIGITAL_INPUT);
    this.limitSwitchRight = new DigitalInput(PortMap.SHOOTER_RIGHT_DIGITAL_INPUT);
    this.shooterPID = new PID(SHOOTER_P, SHOOTER_I, SHOOTER_D);
  }

  public void setAngle(double angle) {
    this.shooterPID.addMeasurement(angle - Robot.get(SD.ShooterAngle));
    setSpeed(this.shooterPID.getOutput());
  }

  // [-1,1]
  public void setSpeed(double speed) {
    this.shooterMotor.set(speed);
  }

  public void updateRobotState() {
    double angle = 0.0;
    if      (limitSwitchLeft.get()) {angle = LEFT_EXTREME_ANGLE;}
    else if (limitSwitchRight.get()){angle = RIGHT_EXTREME_ANGLE;}
    else {angle = this.rotateTalon.getWheelAngle();}
    this.rotateTalon.setWheelAngle(angle);
    Robot.set(SD.ShooterAngle, angle);
  }

  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}