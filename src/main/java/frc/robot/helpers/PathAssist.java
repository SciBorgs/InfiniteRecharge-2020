package frc.robot.helpers;

import java.util.ArrayList;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.helpers.Point;
import frc.robot.pathFollowing.PathSmoother;
import frc.robot.localization.EncoderLocalization;

public class PathAssist{
    private int lastPointHitIndex;
    private double distanceTolerance = 0;
    double numPoints;
    ArrayList<Point> path;
    public EncoderLocalization encoderLocalization;

    public PathAssist() {}
    
    public void setPath(ArrayList<Point> path, int numPoints) { 
        PathSmoother smoother = new PathSmoother(path, numPoints);
        Utils.deepCopy(smoother.getFinalPath(), this.path);
    }
    
    public void update(Point currPos, double heading) {
        int lastPointHit = getLastPointHitIndex(currPos);
        double desiredHeading = getDesiredHeading(this.path.get(lastPointHit), this.path.get(lastPointHit + 1));
        double dtheta = desiredHeading - heading;
        
        double turnMagnitude = 0;
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
    }

    private double getDesiredHeading(Point p1, Point p2) { return Math.atan2(p1.y - p2.y, p1.x - p2.x); }

    private int getLastPointHitIndex(Point currPos) {
		int pointToHit = lastPointHitIndex + 1;
		double distance = CoordinateSystemProcessing.getDistance(this.path.get(pointToHit), currPos);
		if (distance < distanceTolerance) { lastPointHitIndex = pointToHit; }
		return lastPointHitIndex;
    }
    
    public boolean isDone() {
		Point currPos = new Point(encoderLocalization.getX(), encoderLocalization.getY());
		return getLastPointHitIndex(currPos) == (this.path.size() - 1);
    }
}