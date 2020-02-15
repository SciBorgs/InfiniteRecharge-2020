package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
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
  private static final double LIMELIGHT_OFFSET = -Utils.inchesToMeters(9.0);
  private static final double BALL_TO_OUTER_PORT_DISTANCE = Robot.get(SD.DistanceToPort) - LIMELIGHT_OFFSET;
  private static final double BALL_TO_INNER_PORT_DISTANCE = BALL_TO_OUTER_PORT_DISTANCE + 0.762;
  private static final double BALL_TO_FLOOR_BOTTOM_HEIGHT = 0.571;
  private static final double BALL_TO_PORT_CENTER_HEIGHT = 2.49555 - BALL_TO_FLOOR_BOTTOM_HEIGHT;

  private static final double MINIMUM_HOOD_ANGLE = Math.toRadians(25);
  private static final double MAXIMUM_HOOD_ANGLE = Math.toRadians(65);

  private static final double OUTER_PORT_ERROR_P_GAIN = 0.35;

  private static final double OUTER_PORT_HEIGHT_TOLERANCE = 0.01;
  private static final double INNER_PORT_HEIGHT_TOLERANCE = 0.1;

  private static double outerPortError = Double.POSITIVE_INFINITY;
  private static double innerPortError = Double.POSITIVE_INFINITY;

  public static double ballVelocity = 0;

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

  public static double getMotorRPM() {
    return 60 * (ballVelocity * 2) / (2 * Math.PI * BALL_RADIUS);
  }

  public static void shoot() {Robot.shooterSubsystem.setShooterSpark(Robot.shooterSubsystem.RPMToOmega(getMotorRPM()));}

  public static double getHoodAngle() {
    double a = 0.843819;
    double b = -14.4381;
    double c = 81.5331;
    double angle = Math.toRadians(Math.pow(a, 2) * BALL_TO_OUTER_PORT_DISTANCE * b 
                                                 * BALL_TO_OUTER_PORT_DISTANCE + c);
    return Utils.limitOutput(angle, MAXIMUM_HOOD_ANGLE, MINIMUM_HOOD_ANGLE);
  }

  public static void setHoodAngle() {Robot.shooterSubsystem.setHoodSpark(getHoodAngle());}

  private static Function<Double, Double> getTrajectoryFunction() {
    double hoodAngle = getHoodAngle();
    if (ballVelocity == 0) {
      ballVelocity =
          Math.sqrt(
              (BALL_TO_INNER_PORT_DISTANCE * GRAVITATIONAL_ACCELERATION) / Math.sin(2 * hoodAngle));
    } else {
      double increment = Math.abs(outerPortError * OUTER_PORT_ERROR_P_GAIN); 
      if (innerPortError > 0) {ballVelocity += increment;} 
      else                    {ballVelocity -= increment;}
    }
    
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
