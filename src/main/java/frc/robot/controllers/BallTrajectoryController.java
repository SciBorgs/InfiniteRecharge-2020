package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import java.util.function.Function;

public class BallTrajectoryController {
  private static final double GRAVITATIONAL_ACCELERATION = 9.81;
  private static final double AIR_DENSITY = 1.21;

  private static final double BALL_RADIUS = 0.0889;
  private static final double BALL_MASS = 0.12;
  private static final double BALL_CROSS_SECTIONAL_AREA = Math.PI * Math.pow(BALL_RADIUS, 2);
  private static final double BALL_LIFT_COEFFICIENT = 0.01;
  private static final double BALL_DRAG_COEFFICIENT = 0.61;
  private static final double LIMELIGHT_OFFSET = 0;
  private static final double BALL_TO_OUTER_PORT_DISTANCE = Robot.get(SD.DistanceToPort) - LIMELIGHT_OFFSET;
  private static final double BALL_TO_INNER_PORT_DISTANCE = BALL_TO_OUTER_PORT_DISTANCE + 0.762;
  private static final double BALL_TO_FLOOR_BOTTOM_HEIGHT = 0.571;
  private static final double BALL_TO_PORT_CENTER_HEIGHT = 2.49555 - BALL_TO_FLOOR_BOTTOM_HEIGHT;
  private static final double BALL_VELOCITY_INCREMENT = 0.01;

  private static final double OUTER_PORT_HEIGHT_TOLERANCE = 0.18;
  private static final double INNER_PORT_HEIGHT_TOLERANCE = 0.01;

  private static double outerPortError = Double.POSITIVE_INFINITY;
  private static double innerPortError = Double.POSITIVE_INFINITY;

  private static double ballVelocity = 0;

  public static void optimizeParameters() {
    Function<Double, Double> trajectoryFunction = getTrajectoryFunction();
    outerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_OUTER_PORT_DISTANCE);
    innerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_INNER_PORT_DISTANCE);
  }

  public static boolean areParametersOptimal() {
    return Math.abs(outerPortError) <= OUTER_PORT_HEIGHT_TOLERANCE
        && Math.abs(innerPortError) <= INNER_PORT_HEIGHT_TOLERANCE;
  }

  public static void shoot() {
    Robot.shooterSubsystem.setShooterSpark(getMotorRPM());
    //Robot.shooterSubsystem.setShooterSpark(getMotorRPM());
  }

  public static double getMotorRPM() {
    return 60 * (ballVelocity * 2) / (2 * Math.PI * BALL_RADIUS);
  }

  public static void setHoodAngle() {
    System.out.println("HOOD GOING TO: " + getHoodAngle());
    Robot.shooterSubsystem.setHoodSpark(getHoodAngle());
  }

  public static double getHoodAngle() {
    double angle = Math.toRadians(0.958134 * Math.pow(BALL_TO_OUTER_PORT_DISTANCE, 2) 
    + (-15.0864 * BALL_TO_OUTER_PORT_DISTANCE) + 86.285);
    System.out.println("ANGLE " + angle);
    if (angle < Math.toRadians(25))
      return Math.toRadians(25);
    if (angle > Math.toRadians(65))
      return Math.toRadians(65);
    return angle;
  }

  private static Function<Double, Double> getTrajectoryFunction() {
    double hoodAngle = getHoodAngle();
    if (ballVelocity == 0.0) {
      ballVelocity =
          Math.sqrt(
              (BALL_TO_INNER_PORT_DISTANCE * GRAVITATIONAL_ACCELERATION) / Math.sin(2 * hoodAngle));
    }
    if (innerPortError > 0) {ballVelocity += BALL_VELOCITY_INCREMENT;} 
    else                    {ballVelocity -= BALL_VELOCITY_INCREMENT;}
    System.out.println("BALLV " + ballVelocity);

    double vx = ballVelocity * Math.cos(hoodAngle);
    double vy = ballVelocity * Math.sin(hoodAngle);
    double k = (AIR_DENSITY * BALL_CROSS_SECTIONAL_AREA * ballVelocity) / (2 * BALL_MASS * vx);
    double a = -k * (BALL_DRAG_COEFFICIENT * vx + BALL_LIFT_COEFFICIENT * vy);
    double c =
        (k * (BALL_LIFT_COEFFICIENT * vx - BALL_DRAG_COEFFICIENT * vy))
            - (GRAVITATIONAL_ACCELERATION / vx);
    double i = -(((a * vy - vx * c) * Math.log(Math.abs(vx))) / Math.pow(a, 2));
    return x ->
        ((a * vy - vx * c) * Math.log(Math.abs(a * x + vx))) / Math.pow(a, 2) + (c * x) / a + i;
  }
}
