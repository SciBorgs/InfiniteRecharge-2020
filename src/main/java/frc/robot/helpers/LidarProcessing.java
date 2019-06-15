package frc.robot.helpers;

import frc.robot.Utils;

import java.util.Hashtable;
import frc.robot.subsystems.LidarServer;
import frc.robot.subsystems.LidarServer.LidarScanInfo;

// FILE HAS NOT BEEN CLEANED UP //
class Line{
    public double m, b;
    public Line(double m_des, double b_des) {m = m_des; b = b_des;}
}

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
    public static Point   center(Point p1, Point p2)            {return new Point((p1.x + p2.x)/2,(p1.y + p2.y)/2);}
    public static Line    pointSlopeForm(Point p, double m)     {return new Line(m, p.y - m * p.x);}
    public static double  yOf(Line l, double x)                 {return l.m * x + l.b;}
    public static Line    twoPointForm(Point p1, Point p2)      {return pointSlopeForm(p1,(p2.y - p1.y)/(p2.x - p1.x));}
    public static double  distance(Point p1, Point p2)          {return Math.sqrt(Math.pow((p1.x - p2.x),2) + Math.pow((p1.y - p2.y),2));}   
    public static Point   pointOfX(Line l, double x)            {return new Point(x,yOf(l,x));}
    public static Point   intersection(Line l1, Line l2)        {return pointOfX(l1, (l1.b - l2.b)/(l2.m - l1.m));}
    public static Line    perpindicular(Line l, Point p)        {return pointSlopeForm(p, -1/l.m);}
    public static double  distanceToLine(Line l, Point p)       {return distance(p,intersection(perpindicular(l,p),l));}
    public static double  length(Point p)                       {return distance(p,new Point(0,0));}
    public static boolean isOrigin(Point p)                     {return length(p) == 0;}
    public static Point   angleOnLine(double theta, Line l)     {return intersection(l, pointSlopeForm(new Point(0,0), Math.tan(theta)));}
    
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
        Line wall = twoPointForm(points[angleToIndex(a1)],points[angleToIndex(a2)]);
        return Math.atan(wall.m);
    }

    public static void displayPoints(Point[] points){
        //sMain.addPoints(points);
    }
}
