package frc.robot.helpers;

import java.util.ArrayList;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.helpers.PID;
import frc.robot.helpers.Point;
import frc.robot.helpers.PathSmoother;

public class PathAssist {

  private static final double TURN_P = 0, TURN_I = 0, TURN_D = 0;
  private PID turnPID = new PID(TURN_P, TURN_I, TURN_D);
  
  private int lastPointHitIndex = 0;
  private double distanceTolerance = 0;
  private double rightVel, leftVel, prevRightVel, prevLeftVel;
  private ArrayList<Point> path;
  
  // for rateLimiter
  private double maxAccel = 0; // through testing
  private static final double UPDATE_TIME = 0; // refresh time
  private boolean right = true;
  private boolean left = !right;

  public PathAssist() {}

  public void setPath(ArrayList<Point> path, int numPoints) {
    reset();
    PathSmoother smoother = new PathSmoother(path, numPoints);
    Utils.deepCopy(smoother.getFinalPath(), this.path);
  }

  public void reset() {
    this.lastPointHitIndex = 0;
    this.leftVel = 0;
    this.rightVel = 0;
    this.prevLeftVel = 0;
    this.prevRightVel = 0;
  }

  public void update(Point currPos, double heading) {
    int lastPointHit = getLastPointHitIndex(currPos);
    double desiredHeading = getDesiredHeading(this.path.get(lastPointHit), this.path.get(lastPointHit + 1));
    double error = desiredHeading - heading;
    turnPID.addMeasurement(error);
    double turnMagnitude = turnPID.getOutput();

    double forward = (Robot.driveSubsystem.processStick(Robot.oi.leftStick) + Robot.driveSubsystem.processStick(Robot.oi.rightStick)) / 2;
    double leftPower  = rateLimiter( (forward * (1 + turnMagnitude)), maxAccel, left);
    double rightPower = rateLimiter( (forward * (1 - turnMagnitude)), maxAccel, right);

    Robot.driveSubsystem.setSpeedTank(leftPower, rightPower);
  }

  private double getDesiredHeading(Point p1, Point p2) { return Math.atan2(p1.y - p2.y, p1.x - p2.x); }

  // limits acceleration
  private double rateLimiter(double input, double maxRate, boolean right) {
    double maxChange = UPDATE_TIME * maxRate;
    if (right) {
      this.rightVel += Utils.limitOutput(input - this.prevRightVel, maxChange);
      this.prevRightVel = this.rightVel;
      return rightVel;
    } else {
      this.leftVel += Utils.limitOutput(input - this.prevLeftVel, maxChange);
      this.prevLeftVel = this.leftVel;
      return leftVel;
    }
  }

  private int getLastPointHitIndex(Point currPos) {
    int pointToHit = lastPointHitIndex + 1;
    double distance = CoordinateSystemProcessing.getDistance(this.path.get(pointToHit), currPos);
    if (distance < distanceTolerance) {
      lastPointHitIndex = pointToHit;
    }
    return lastPointHitIndex;
  }

  public boolean isDone() {
    Point currPos = new Point(Robot.encoderLocalization.getX(), Robot.encoderLocalization.getY());
    return getLastPointHitIndex(currPos) == (this.path.size() - 1);
  }
}