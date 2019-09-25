package frc.robot.helpers;

import frc.robot.Utils;

import java.util.Hashtable;
import frc.robot.subsystems.LidarServer;
import frc.robot.subsystems.LidarServer.LidarScanInfo;

// FILE HAS NOT BEEN CLEANED UP //

public class LidarProcessing{
    public LidarProcessing() {
    }

    public static final double FIRST_ANGLE = 0; // This is assuming the first piece of data is backwards
    public static final double FINAL_ANGLE = 360;
    public static final double DELTA_THETA = 1;
    public static final double GAP = 8 / Utils.METERS_TO_INCHES;
    public static final double GAP_OFFSET = .0;
    public static final double GAP_PRECISION = .15;
    public static final double LIDAR_SHIFT = 0; // In meters away from the center

    public static Point   toPoint(double l, double theta)       {return new Point(l * Math.cos(theta), l * Math.sin(theta));}
    public static Point   toPointDegrees(double l, double theta){return toPoint(l,Math.toRadians(theta));}
    
    public static double makeAngleInRange(double angle){
        while(angle < FIRST_ANGLE){angle += FINAL_ANGLE;}
        return angle % FINAL_ANGLE;
    }
    public static int makeIndexInRange(int i, int max){
        while(i < 0){i += max;}
        return i % max;
    }

    private static double getDistance(Hashtable<Double,Hashtable<LidarScanInfo,Double>> polarPoints, double roundedAngle){
        return polarPoints.get(roundedAngle).get(LidarScanInfo.Distance);
    } 
    private static double getAngle(Hashtable<Double, Hashtable<LidarScanInfo, Double>> polarPoints, double roundedAngle) {
        return polarPoints.get(roundedAngle).get(LidarScanInfo.RealAngle);
    }

    public static Point[] polarHashToPoints(Hashtable<Double,Hashtable<LidarScanInfo,Double>> polarPoints){
        int length = (int) ((FINAL_ANGLE - FIRST_ANGLE) / DELTA_THETA);
        Point[] points = new Point[length];
        for(int i = 0; i < length; i++){
            double roundedAngle = indexToAngle(i);
            double distance = getDistance(polarPoints, roundedAngle);
            double angle    = getAngle   (polarPoints, roundedAngle);
            points[i] = toPointDegrees(distance, angle);
        }
        return points;
    }

    public static Point[] fetchScan(){
        return polarHashToPoints(LidarServer.getInstance().lidarScan);
    }

    public static void printScan(){
        Hashtable<Double, Hashtable<LidarScanInfo, Double>> data = LidarServer.getInstance().lidarScan;
        
        System.out.println("size: " + data.size());
        if (data.size() > 0){
            System.out.print("[");
            for (double angle = FIRST_ANGLE; angle < FINAL_ANGLE; angle = nextAngle(angle)){
                if (data.containsKey(angle))
                    System.out.print("(" + angle + "," + data.get(angle) + "), ");
            }
            System.out.println("]");
        }
    }

    public static int angleToIndex(double angle){
        return (int) ((angle % FINAL_ANGLE + FIRST_ANGLE % FINAL_ANGLE) / DELTA_THETA);
    }
    public static double indexToAngle(int index){
        return index * DELTA_THETA + FIRST_ANGLE;
    }
    public static double nextAngle(double angle){
        return makeAngleInRange(angle + DELTA_THETA);
    }
    public static int nextIndex(int index, int max){
        return makeIndexInRange(index + 1, max);
    }
    public static int prevIndex(int index, int max){
        return makeIndexInRange(index - 1, max);
    }
    public static double wallRotation(double a1, double a2){
        Point[] points = fetchScan();
        Line wall = new Line(points[angleToIndex(a1)],points[angleToIndex(a2)]);
        return Geo.thetaOf(wall);
    }

    public static void displayPoints(Point[] points){
        //sMain.addPoints(points);
    }
}
