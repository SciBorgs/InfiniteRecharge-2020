package frc.robot.auto;

import java.util.ArrayList;
import frc.robot.helpers.Vector;

// curvature -> problem 1 : x1 = x2 results in divide by zero
//              problem 2 : radius very large, ie curvature is 0, and path is a straight line
public class PathInit {
    private ArrayList<Waypoint> robotPath = new ArrayList<>();
    
    //initializes the path with velocity and curvature
    public void initPath(double maxVel, double maxAccel, double turningTolerance) {
		setCurvature();
		setTargetVelocity(maxAccel, maxVel, turningTolerance);
    }
    
    // find curvature of each point in path - excluding first and last point
    public void setCurvature() {
        double side1, side2, side3, productOfSides, semiperimeter, area, r, curvature;
        // uses 
        for (int i = 1; i < robotPath.size() - 1; i++) {
            Waypoint point = robotPath.get(i);
            Vector pastPoint = robotPath.get(i - 1).getPosition();
            Vector currPoint = robotPath.get(i).getPosition();
            Vector nextPoint = robotPath.get(i + 1).getPosition();

            side1 = Vector.distanceBetween(pastPoint, currPoint);
            side2 = Vector.distanceBetween(currPoint, nextPoint);
            side3 = Vector.distanceBetween(pastPoint, nextPoint);

            productOfSides = side1 * side2 * side3;
            semiperimeter = .5 * (side1 + side2 + side2);
            area = Math.sqrt(semiperimeter * (semiperimeter - side1) * (semiperimeter - side2) * (semiperimeter - side3));
            
            r = (productOfSides) / (4 * area);
            curvature = 1 / r;
            
            point.setCurvature(curvature);
        }
    }

    // 2 or 3 step process for finding velocity at point
    // 1. find maximum possible velocity at each point
    // 2. add maximum acceleration -> really just used in next step
    // 3. create a target velocity
    public double calcMaxVelocity(double maxVel, double turningTolerance, int index) {
        if(index > 0) {
            double curvature = robotPath.get(index).getCurvature();
            return Math.min(maxVel, turningTolerance / curvature);
        }
        return maxVel;
    }

    // uses simple 2D kinematics: vf = sqrt(vi^2 + 2 * a * d)
    public void setTargetVelocity(double maxAccel, double maxVel, double turningTolerance) {
        double distance, vi, vf;
        robotPath.get(robotPath.size() - 1).setVelocity(0); // want ending velocity to be 0

        for (int i = robotPath.size() - 2; i >= 0; i--) {
            distance = Vector.distanceBetween(robotPath.get(i + 1).getPosition(), robotPath.get(i).getPosition()); 
            vi = robotPath.get(i + 1).getVelocity();
            double maxReachableVel = Math.sqrt(Math.pow(vi, 2) + (2 * maxAccel * distance));
            vf = Math.min(calcMaxVelocity(maxVel, turningTolerance, i), maxReachableVel);
            robotPath.get(i).setVelocity(vf);
        }
    }
}