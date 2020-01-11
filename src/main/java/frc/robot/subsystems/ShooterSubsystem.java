package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {
  private SciSpark testSpark;

  private SciTalon rotateTalon;
  private SciSpark shooterMotorTop;
  private SciSpark shooterMotorBottom;
  
  private final int LEFT_EXTREME_ANGLE = -90;
  private final int RIGHT_EXTREME_ANGLE = 90;

  private DigitalInput limitSwitchLeft;  // -90 deg
  private DigitalInput limitSwitchRight; // 90 deg

  private PID shooterPID;
  private final int SHOOTER_P = 0;
  private final int SHOOTER_I = 0;
  private final int SHOOTER_D = 0;

  public ShooterSubsystem() {
    this.testSpark = new SciSpark(0);

    this.rotateTalon  = new SciTalon(PortMap.SHOOTER_ROTATE_SPARK);
    this.shooterMotorTop = new SciSpark(PortMap.SHOOTER_MOTOR_TOP);
    this.shooterMotorBottom = new SciSpark(PortMap.SHOOTER_MOTOR_BOTTOM);
    this.limitSwitchLeft  = new DigitalInput(PortMap.SHOOTER_LEFT_DIGITAL_INPUT);
    this.limitSwitchRight = new DigitalInput(PortMap.SHOOTER_RIGHT_DIGITAL_INPUT);
    this.shooterPID = new PID(SHOOTER_P, SHOOTER_I, SHOOTER_D);
  }

  public void logTestSparkEncoder() {
    DelayedPrinter.print("WHEEL ANGLE: " + this.testSpark.getWheelAngle());
  }

  public void setAngle(double angle) {
    this.shooterPID.addMeasurement(angle - Robot.get(SD.ShooterAngle));
    setSpeed(this.shooterPID.getOutput());
  }

  // [-1,1]
  public void setSpeed(double speed) {
    this.rotateTalon.set(speed);
  }

  public void updateRobotState() {
    if      (limitSwitchLeft.get()) { this.rotateTalon.setWheelAngle(LEFT_EXTREME_ANGLE);}
    else if (limitSwitchRight.get()){ this.rotateTalon.setWheelAngle(RIGHT_EXTREME_ANGLE);}
    Robot.set(SD.ShooterAngle, this.rotateTalon.getWheelAngle());
  }

  public void setSpeedTop(double value) {
    this.shooterMotorTop.set(value);
  }

  public void setSpeedBottom(double value) {
    this.shooterMotorBottom.set(value);
  }

  @Override
  protected void initDefaultCommand() {
		//IGNORE THIS METHOD
  }
}