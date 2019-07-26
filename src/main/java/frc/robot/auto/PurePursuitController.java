package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.helpers.Point;
import frc.robot.helpers.CoordinateSystemProcessing;

// TODO 
// move future math to sperate file
//
// curvature -> problem 1 : x1 = x2 results in divide by zero
//              problem 2 : radius very large, ie curvature is 0, and path is a straight line
//              problem 3 : is first point on path 0,0 or the next point? could help with creating curvature for first point 
//
//velocity   -> problem 1: figure out starting velocity - current solution is sketchy - rate limiter (for acceleration limited velocity) might fix that
//           -> problem 2: check for out of bounds exception on the for
public class PurePursuitController{
    double  initialVel, maxVel, maxAccel, turningTolerance; // turning tolerance scales how slow you want to go while turning
    Timer timer;

    public PurePursuitController(double initialVel, double maxAccel, double maxVel, double turningTolerance, Timer timer) {
        this.initialVel = initialVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.turningTolerance = turningTolerance;
        this.timer = timer;
    }

    // find curvature of each point in path - excluding first and last point
    // method 2 - https://www.qc.edu.hk/math/Advanced%20Level/circle%20given%203%20points.htm
    public ArrayList<Double> curvature(ArrayList<Point> points) {
        ArrayList<Double> curvatureList = new ArrayList<Double>(points.size() - 2); //size will be 2 less because end points have no curvature
        double x1, x2, x3, y1, y2, y3, k1, k2, a, b, r, curvature;
         
        for (int i = 0; i < curvatureList.size(); i++){
            x1 = points.get(i).x;
            x2 = points.get(i + 1).x;
            x3 = points.get(i + 2).x;
            y1 = points.get(i).y;
            y2 = points.get(i + 1).y;
            y3 = points.get(i + 2).y;

            k1 = .5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2))/(x1 - x2);                                             // k1 = 0.5 * (x1^2 + y1^2 − x2^2 − y2^2)/(x1 − x2)
            k2 = ((y1 - y2)/(x1 - x2));                                                                                                              // k2 = (y1 − y2)/(x1 − x2)
            b  = .5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3, 2) + 2 * x3 * k1 - Math.pow(y3, 2))/(x3 *k2 - y3 +y2 -x2 *k2); // b  = 0.5 * (x2^2 − 2 * x2 * k1 + y2^2 − x3^2 + 2 * x3 * k1 − y3^2)/(x3 * k2 − y3 + y2 − x2 * k2)
            a = k1 - k2 * b;                                                                                                                         // a  = k1 − k2 * b
            r = Math.sqrt(Math.pow((x1 - a), 2) + Math.pow((y1 - b), 2));                                                                            // r  = sqrt((x1 − a)^2 + (y1 − b)^2);
            curvature = 1 / r;                                                                            
            curvatureList.add(curvature);
        }

        return curvatureList;
    }


    //    2 or 3 step process for finding velocity at point
    // 1. find maximum possible velocity at each point
    // 2. add maximum acceleration -> really just used in next step
    // 3. create a target velocity
    public ArrayList<Double> maxVelocity(double initialVel, double maxVel, double turningTolerance, ArrayList<Point> points, ArrayList<Double> curvature){
        double velocity;
        ArrayList<Double> maxVelocity = new ArrayList<Double>(points.size());
        maxVelocity.add(initialVel); // starting velocity
        
        for(int i = 0; i < points.size(); i++){
            velocity = Math.min(maxVel, turningTolerance / curvature.get(i - 1));
            maxVelocity.add(velocity);
        }

        return maxVelocity;
    }

    // uses simple 2D kinematics vf = sqrt(vi^2 + 2 * a * d)
    public ArrayList<Double> targetVelocity(double maxAccel, ArrayList<Point> points, ArrayList<Double> maxVelocity){
        double distance, vi, vf;
        ArrayList<Double> targetVelocity = new ArrayList<Double>(points.size());
        targetVelocity.set(points.size() - 1, 0.0); // want ending velocity to be 0
        
        // this might throw an error
        for(int i = points.size(); i > 0; i--){
            distance = CoordinateSystemProcessing.getDistance(points.get(i), points.get(i - 1)); // theres an error here? removing the Point class from CoordinateSystemProcessing fixes it
            vi = targetVelocity.get(i + 1);
            vf = Math.min(maxVelocity.get(i), Math.sqrt(Math.pow(vi, 2) + 2 * maxAccel * distance));
            targetVelocity.set(i , vf);
        }
        
        return targetVelocity;
    }

    //would be running in real time
    public double rateLimiter(){
        // need times
        //      max acel
        //      and a cap 
    }

}