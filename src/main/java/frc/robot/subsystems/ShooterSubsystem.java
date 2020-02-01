package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.controllers.PID;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder;

import java.util.function.Function;

public class ShooterSubsystem extends Subsystem {
  private final double GRAVITATIONAL_ACCELERATION = 9.81;
  private final double AIR_DENSITY = 1.21;

  private final double BALL_RADIUS = 0.0889;
  private final double BALL_MASS = 0.12;
  private final double BALL_CROSS_SECTIONAL_AREA = Math.PI * Math.pow(BALL_RADIUS, 2);
  private final double BALL_LIFT_COEFFICIENT = 0.01;
  private final double BALL_DRAG_COEFFICIENT = 0.61;
  private final double BALL_TO_OUTER_PORT_DISTANCE = 9;
  private final double BALL_TO_INNER_PORT_DISTANCE = BALL_TO_OUTER_PORT_DISTANCE + 0.74295;
  private final double BALL_TO_PORT_CENTER_HEIGHT = 2.49555 - 0.17145;
  private final double BALL_VELOCITY_INCREMENT = 0.01;

  private final double MAX_HOOD_ANGLE = Math.toRadians(70);
  private final double MIN_HOOD_ANGLE = Math.toRadians(30);
  private final double MAX_HOOD_ANGLE_DISTANCE = 1.166;
  private final double MIN_HOOD_ANGLE_DISTANCE = 6.073;

  private final double OUTER_PORT_HEIGHT_TOLERANCE = 0.4;
  private final double INNER_PORT_HEIGHT_TOLERANCE = 0.3;

  private SciSpark hoodSpark, shooterSpark;
  private PID hoodAnglePID;
  private CANPIDController shooterSparkVelocityPID;
  private double HOOD_SPARK_GEAR_RATIO = 36.0 / 3449;
  private double SHOOTER_SPARK_GEAR_RATIO = 1;

  private SciThroughBoreEncoder absEncoder;

  private static class ShooterData {
    public static double hoodAngle, ballVelocity, motorRPM;
    public static double outerPortError = Double.POSITIVE_INFINITY;
    public static double innerPortError = Double.POSITIVE_INFINITY;
  }

  public ShooterSubsystem() {
    this.hoodSpark    = new SciSpark(PortMap.HOOD_SPARK, HOOD_SPARK_GEAR_RATIO);
    this.shooterSpark = new SciSpark(PortMap.SHOOTER_SPARK, SHOOTER_SPARK_GEAR_RATIO);

    this.hoodAnglePID = new PID(0.01, 0, 0);
    this.shooterSparkVelocityPID = this.shooterSpark.getPIDController();
    this.shooterSparkVelocityPID.setP(0.001);
    this.shooterSparkVelocityPID.setI(0.000001);
    this.shooterSparkVelocityPID.setD(0.01);
    this.shooterSparkVelocityPID.setOutputRange(-1, 1);

    this.absEncoder = new SciThroughBoreEncoder(0);
  }

  public void optimizeParameters() {
    ShooterData.hoodAngle = getHoodAngle();
    ShooterData.ballVelocity = getBallVelocity();
    ShooterData.motorRPM = getMotorRPM();
    Function<Double, Double> trajectoryFunction = getTrajectoryFunction();
    ShooterData.outerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_OUTER_PORT_DISTANCE);
    ShooterData.innerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_INNER_PORT_DISTANCE);
  }

  public boolean areParametersOptimal() {
    return Math.abs(ShooterData.outerPortError) <= OUTER_PORT_HEIGHT_TOLERANCE
        && Math.abs(ShooterData.innerPortError) <= INNER_PORT_HEIGHT_TOLERANCE;
  }

  public void logParamters() {
    System.out.println("RPM " + this.shooterSpark.getEncoder().getVelocity());
  }

  public void setHoodAngle() {
    this.hoodAnglePID.addMeasurement(ShooterData.hoodAngle - this.absEncoder.getRadians());
    this.hoodSpark.set(this.hoodAnglePID.getOutput());
  }

  public void shoot() {this.shooterSparkVelocityPID.setReference(ShooterData.motorRPM, ControlType.kVelocity);}

  private double getHoodAngle() {
    if (BALL_TO_OUTER_PORT_DISTANCE < MAX_HOOD_ANGLE_DISTANCE){return MAX_HOOD_ANGLE;}
    if (BALL_TO_OUTER_PORT_DISTANCE > MIN_HOOD_ANGLE_DISTANCE){return MIN_HOOD_ANGLE;}
    double a = 0.958134;
    double b = -15.0864;
    double c = 86.285;
    return Math.toRadians(
        a * Math.pow(BALL_TO_OUTER_PORT_DISTANCE, 2) + (b * BALL_TO_OUTER_PORT_DISTANCE) + c);
  }

  private double getMotorRPM() {return 60 * (ShooterData.ballVelocity * 2) / (2 * Math.PI * BALL_RADIUS);}

  private double getBallVelocity() {
    if (ShooterData.ballVelocity == 0.0) {
      return Math.sqrt(
          (BALL_TO_INNER_PORT_DISTANCE * GRAVITATIONAL_ACCELERATION)
              / Math.sin(2 * ShooterData.hoodAngle));
    }
    if (ShooterData.innerPortError > 0) {return ShooterData.ballVelocity + BALL_VELOCITY_INCREMENT;}
    return ShooterData.ballVelocity - BALL_VELOCITY_INCREMENT;
  }

  private Function<Double, Double> getTrajectoryFunction() {
    double vx = ShooterData.ballVelocity * Math.cos(ShooterData.hoodAngle);
    double vy = ShooterData.ballVelocity * Math.sin(ShooterData.hoodAngle);
    double k =
        (AIR_DENSITY * BALL_CROSS_SECTIONAL_AREA * ShooterData.ballVelocity) / (2 * BALL_MASS * vx);
    double a = -k * (BALL_DRAG_COEFFICIENT * vx + BALL_LIFT_COEFFICIENT * vy);
    double c =
        (k * (BALL_LIFT_COEFFICIENT * vx - BALL_DRAG_COEFFICIENT * vy))
            - (GRAVITATIONAL_ACCELERATION / vx);
    double i = -(((a * vy - vx * c) * Math.log(Math.abs(vx))) / Math.pow(a, 2));
    return x ->
        ((a * vy - vx * c) * Math.log(Math.abs(a * x + vx))) / Math.pow(a, 2) + (c * x) / a + i;
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
  }
}
