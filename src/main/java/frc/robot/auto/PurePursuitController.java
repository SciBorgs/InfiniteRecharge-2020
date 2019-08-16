package frc.robot.auto;

import java.util.ArrayList;
import java.util.Optional;
import frc.robot.helpers.Vector;
import frc.robot.helpers.Waypoint;
import frc.robot.localization.EncoderLocalization;

public class PurePursuitController{
    private static PurePursuitController instance;
	private int lastClosestPoint;
    private ArrayList<Path> paths = new ArrayList<>(); // allows mulitple paths to be pieced together
	private Path path;
	private double lookaheadDistance;
	private double robotWidth = 0;
    private double feedbackMultiplier = 0;
    public EncoderLocalization encoderLocalization;

	private PurePursuitController() { reset(); }

	public static PurePursuitController getInstance() {
		return instance == null ? instance = new PurePursuitController() : instance;
	}

    public void setPaths(ArrayList<Path> paths, double lookaheadDistance) {
		reset();
		this.paths = paths;
		this.lookaheadDistance = lookaheadDistance;
	}

	public void setRobotWidth(double robotWidth) {
		this.robotWidth = robotWidth;
	}

	public void setFeedbackMultiplier(double feedbackMultiplier) {
		this.feedbackMultiplier = feedbackMultiplier;
	}

	public void reset() {
		this.lastClosestPoint = 0;
	}

    // updates the power delivered to the motors
	public DriveBase update(Vector currPos, double currLeftVel, double currRightVel, double heading) {
		boolean onLastSegment = false;
		int closestPointIndex = getClosestPointIndex(currPos);
		Vector lookaheadPoint = new Vector(0, 0);
		ArrayList<Waypoint> robotPath = path.getRobotPathWaypoints();
		for (int i = closestPointIndex + 1; i < robotPath.size(); i++) {
			Vector startPoint = robotPath.get(i - 1).getPosition();
			Vector endPoint = robotPath.get(i).getPosition();
            
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
		double leftTargetVel = calculateLeftTargetVelocity(robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);
		double rightTargetVel = calculateRightTargetVelocity(robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);

		double leftFeedback = feedbackMultiplier * (leftTargetVel - currLeftVel);
		double rightFeedback = feedbackMultiplier * (rightTargetVel - currRightVel);

        double leftPower = leftTargetVel + leftFeedback;
        double rightPower = rightTargetVel + rightFeedback;

        return new DriveBase(leftPower, rightPower);
	}

	private double calculateLeftTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 + (robotWidth * curvature))) / 2;
	}

	private double calculateRightTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 - (robotWidth * curvature))) / 2;
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
		return getClosestPointIndex(new Vector(encoderLocalization.getX(), encoderLocalization.getY())) == path.getRobotPathWaypoints().size() - 1;
    }
    
}