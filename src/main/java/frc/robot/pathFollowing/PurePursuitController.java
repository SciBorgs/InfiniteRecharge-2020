package frc.robot.pathFollowing;

import java.util.ArrayList;
import java.util.Optional;
import frc.robot.Utils;
import frc.robot.helpers.Vector;
import frc.robot.helpers.Waypoint;
import frc.robot.localization.EncoderLocalization;

public class PurePursuitController{
    private static PurePursuitController instance;
	private int lastClosestPoint;
	private Path path;
	private double lookaheadDistance;
	private double robotWidth = 0;
	private double updateTime; // refresh time
	private double maxAccel;
	private double kp, kv, ka;
	private double rightVel, leftVel, prevRightVel, prevLeftVel;
	private boolean right = true;  // for rateLimiter
	private boolean left  = false;
    public EncoderLocalization encoderLocalization;

	private PurePursuitController() { reset(); }

	public static PurePursuitController getInstance() {	return instance == null ? instance = new PurePursuitController() : instance; }

	public void setPath(Path path, double lookaheadDistance) {
		reset();
		this.path = path;
		this.lookaheadDistance = lookaheadDistance;
	}

	public void setRobotWidth(double robotWidth) { this.robotWidth = robotWidth; }
	public void reset() { 
		this.lastClosestPoint = 0;
		this.leftVel  = 0;
		this.rightVel = 0;
		this.prevLeftVel = 0;
		this.prevRightVel = 0;
	}

    // updates the power delivered to the motors
	public DriveBase update(Vector currPos, double heading) {
		boolean onLastSegment = false;
		int closestPointIndex = getClosestPointIndex(currPos);
		Vector lookaheadPoint = new Vector(0, 0);
		ArrayList<Waypoint> robotPath = path.getRobotPathWaypoints();

		for (int i = closestPointIndex + 1; i < robotPath.size(); i++) {
			Vector startPoint = robotPath.get(i - 1).getPosition();
			Vector endPoint   = robotPath.get(i).getPosition();
            if (i == robotPath.size() - 1){
                onLastSegment = true;
            }
			Optional<Waypoint> lookaheadPtOptional = calculateLookAheadPoint(startPoint, endPoint, currPos, lookaheadDistance, onLastSegment);
			if (lookaheadPtOptional.isPresent()) {
				lookaheadPoint = lookaheadPtOptional.get().getPosition();
				break;
			}
		}

		double curvature = path.calculateCurvatureLookAhead(currPos, heading, lookaheadPoint, lookaheadDistance);
		double leftTargetVel  = calculateLeftTargetVelocity( robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);
		double rightTargetVel = calculateRightTargetVelocity(robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);

		// feedforward 
		double rightFF = calculateFeedForward(rightTargetVel, this.rightVel, this.right);
		double leftFF  = calculateFeedForward(leftTargetVel,  this.leftVel,  this.left);
		
		// feedback 
        double rightFB = calculateFeedback(rightTargetVel, this.rightVel);
		double leftFB  = calculateFeedback(leftTargetVel,  this.leftVel );
		
		// final output
        double leftPower  = leftFF  + leftFB;
        double rightPower = rightFF + rightFB;
		
        return new DriveBase(leftPower, rightPower);
	}

	private double calculateLeftTargetVelocity(double targetRobotVelocity, double curvature) { return targetRobotVelocity * ((2 + (robotWidth * curvature))) / 2; }

	private double calculateRightTargetVelocity(double targetRobotVelocity, double curvature) {	return targetRobotVelocity * ((2 - (robotWidth * curvature))) / 2; }

	private double calculateFeedForward(double targetVel, double currVel, boolean right) {
        double targetAccel = (targetVel - currVel)/(updateTime);
		targetAccel = Utils.limitOutput(targetAccel, maxAccel);
        double rateLimitedVel = rateLimiter(targetVel, maxAccel, right);
        return (kv * rateLimitedVel) + (ka * targetAccel);
	}
	
    private double calculateFeedback(double targetVel, double currVel) { return kp * (targetVel - currVel); }
	
	private double rateLimiter(double input, double maxRate, boolean right) {
        double maxChange = updateTime * maxRate;
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

    /*      Calculates the intersection t-value between a line and a circle, using quadratic formula
     *      A is the starting point,
     *      B is the end point of the ray,
     *      C is the center of robot
     *      r is the lookahead point ie the radius
     *  Compute:
     *      d = B - A ( Direction vector of ray, from start to end )
     *      f = A - C ( Vector from center sphere to ray start )
     *  Then the intersection is found by plugging ...
     *      P = E + t * d
     *  This is a parametric equation:
     *      Px = E_x + t * d_x
     *      Py = E_y + t * d_y
     *      into
     *      (x - h)^2 + (y - k)^2 = r^2
     *      x = E_x + t * d_x
     *      y = E_y + t * d_y
     *      t: from 0-1, how far along the segment the intersection is
     */

	private Optional<Double> calcIntersectionTVal(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {
		Vector d = Vector.sub(endPoint, startPoint);
		Vector f = Vector.sub(startPoint, currPos);

		double a = d.dot(d);
		double b = 2 * f.dot(d);
		double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
		double discriminant = Math.pow(b, 2) - (4 * a * c);

		if (discriminant < 0) {
			return Optional.empty();
		} else {
			discriminant = Math.sqrt(discriminant);
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			if (t1 >= 0 && t1 <= 1) {
				return Optional.of(t1);
			}
			if (t2 >= 0 && t2 <= 1) {
				return Optional.of(t2);
			}
		}
		return Optional.empty();
	}

	//uses the calculated intersection point to get a Vector value on the path that is the lookahead point
	private Optional<Waypoint> calculateLookAheadPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance, boolean onLastSegment) {
		Optional<Double> tIntersect = calcIntersectionTVal(startPoint, endPoint, currPos, lookaheadDistance);
		if (!tIntersect.isPresent() && onLastSegment) {
			return Optional.of(path.getRobotPathWaypoints().get(path.getRobotPathWaypoints().size() - 1));
		} else if (!tIntersect.isPresent()) {
			return Optional.empty();
		} else {
			Vector intersectVector = Vector.sub(endPoint, startPoint);
			Vector vectorSegment = Vector.scale(intersectVector, tIntersect.get());
			Waypoint point = new Waypoint(Vector.add(startPoint, vectorSegment));
			return Optional.of(point);
		}
	}

    // Calculates the index of the point on the path that is closest to the robot's current position
	private int getClosestPointIndex(Vector currPos) {
		double shortestDistance = Double.MAX_VALUE;
		int closestPoint = 0;
		ArrayList<Waypoint> robotPath = path.getRobotPathWaypoints();
		for (int i = lastClosestPoint; i < robotPath.size(); i++) {
			if (Vector.distanceBetween(robotPath.get(i).getPosition(), currPos) < shortestDistance) {
				closestPoint = i;
				shortestDistance = Vector.distanceBetween(robotPath.get(i).getPosition(), currPos);
			}
		}
		lastClosestPoint = closestPoint;
		return closestPoint;
	}

    // checks to see if current position is close to the final point on the path
	public boolean isDone() {
		Vector currPos = new Vector(encoderLocalization.getX(), encoderLocalization.getY());
		return getClosestPointIndex(currPos) == path.getRobotPathWaypoints().size() - 1;
    }
    
}