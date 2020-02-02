package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import java.util.function.Function;

public class TrajectoryController {
  private static final double GRAVITATIONAL_ACCELERATION = 9.81;
  private static final double AIR_DENSITY = 1.21;

  private static final double BALL_RADIUS = 0.0889;
  private static final double BALL_MASS = 0.12;
  private static final double BALL_CROSS_SECTIONAL_AREA = Math.PI * Math.pow(BALL_RADIUS, 2);
  private static final double BALL_LIFT_COEFFICIENT = 0.01;
  private static final double BALL_DRAG_COEFFICIENT = 0.61;
  private static final double BALL_TO_OUTER_PORT_DISTANCE = 9;
  private static final double BALL_TO_INNER_PORT_DISTANCE = BALL_TO_OUTER_PORT_DISTANCE + 0.74295;
  private static final double BALL_TO_FLOOR_BOTTOM_HEIGHT = 0.3048 + 0.18288;
  private static final double BALL_TO_PORT_CENTER_HEIGHT = 2.49555 - BALL_TO_FLOOR_BOTTOM_HEIGHT;
  private static final double BALL_VELOCITY_INCREMENT = 0.01;

  private static final double MAX_HOOD_ANGLE = Math.toRadians(70);
  private static final double MIN_HOOD_ANGLE = Math.toRadians(30);
  private static final double MAX_HOOD_ANGLE_DISTANCE = 1.166;
  private static final double MIN_HOOD_ANGLE_DISTANCE = 6.073;

  private static final double OUTER_PORT_HEIGHT_TOLERANCE = 0.4;
  private static final double INNER_PORT_HEIGHT_TOLERANCE = 0.3;

  private static double outerPortError = Double.POSITIVE_INFINITY;
  private static double innerPortError = Double.POSITIVE_INFINITY;

  private static double ballVelocity = 0;

  public static void optimizeParameters() {
    Robot.set(SD.HoodSparkWheelAngle, getHoodAngle());

    Function<Double, Double> trajectoryFunction = getTrajectoryFunction();
    outerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_OUTER_PORT_DISTANCE);
    innerPortError =
        BALL_TO_PORT_CENTER_HEIGHT - trajectoryFunction.apply(BALL_TO_INNER_PORT_DISTANCE);

    Robot.set(SD.ShooterSparkRPS, ballVelocity / BALL_RADIUS * 2);
  }

  public static boolean areParametersOptimal() {
    return Math.abs(outerPortError) <= OUTER_PORT_HEIGHT_TOLERANCE
        && Math.abs(innerPortError) <= INNER_PORT_HEIGHT_TOLERANCE;
  }

  private static double getHoodAngle() {
    if (BALL_TO_OUTER_PORT_DISTANCE < MAX_HOOD_ANGLE_DISTANCE) {return MAX_HOOD_ANGLE;}
    if (BALL_TO_OUTER_PORT_DISTANCE > MIN_HOOD_ANGLE_DISTANCE) {return MIN_HOOD_ANGLE;}
    double a = 0.958134;
    double b = -15.0864;
    double c = 86.285;
    return Math.toRadians(
        a * Math.pow(BALL_TO_OUTER_PORT_DISTANCE, 2) + (b * BALL_TO_OUTER_PORT_DISTANCE) + c);
  }

  private static Function<Double, Double> getTrajectoryFunction() {
    double hoodAngle = Robot.get(SD.HoodSparkWheelAngle);
    if (ballVelocity == 0.0) {
      ballVelocity =
          Math.sqrt(
              (BALL_TO_INNER_PORT_DISTANCE * GRAVITATIONAL_ACCELERATION) / Math.sin(2 * hoodAngle));
    }
    if (innerPortError > 0) {ballVelocity += BALL_VELOCITY_INCREMENT;} 
    else                    {ballVelocity -= BALL_VELOCITY_INCREMENT;}

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
