package frc.robot.helpers;

public class Circle {
    
    Point  center;
    double radius;

    public Circle (Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }
    public static Circle makeCircleTangentAnd2Points(Point currPos, double currHeading, Point finalPos) {
       double rotateAngle, kRotated, hRotated, radius;
        if (currHeading < Math.PI / 2) {
            rotateAngle = (Math.PI / 2) - currHeading;
        } else {
            rotateAngle = currHeading - (Math.PI / 2);
        }
        Geo.rotatePoint(currPos, rotateAngle);
        Geo.rotatePoint(finalPos, rotateAngle);
        kRotated = calculateK(currPos, finalPos);
        hRotated = calculateH(currPos);
        radius = calculateR(kRotated, currPos);
        Point center  = new Point(kRotated, hRotated);
        Geo.rotatePoint(center, -1 * rotateAngle);
        return new Circle(center, radius);
    }

    public static double calculateK (Point currPosRotated, Point finalPosRotated) {
        double x0, y0, x1, y1;
        x0 = currPosRotated.x;
        y0 = currPosRotated.y;
        x1 = finalPosRotated.x;
        y1 = finalPosRotated.y;
        double a = (x0 * x0) - (x1 * x1) - ((y1 - y0) * (y1 - y0));
        double b = 2 * (x1 - x0);
        return a / b;
    }
    public static double calculateH (Point currPosRotated) { return currPosRotated.y; }
    public static double calculateR (double k, Point currPosRotated){ return currPosRotated.x - k; }

}