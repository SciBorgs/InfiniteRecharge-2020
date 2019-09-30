package frc.robot.helpers;

public class Circle {
    
    Point  center;
    double radius;

    public Circle (Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public static Circle makeCircleWithTangentAnd2Points(Point currPos, double currHeading, Point finalPos) {
        double rotateAngle, kRotated, hRotated, radius;

        rotateAngle = -1 *  currHeading;        
        Geo.rotatePoint(currPos, rotateAngle);
        Geo.rotatePoint(finalPos, rotateAngle);

        hRotated = calculateH(currPos, finalPos);
        kRotated = calculateK(currPos);
        radius   = calculateR(kRotated, currPos);

        Point center  = new Point(kRotated, hRotated);
        Geo.rotatePoint(center, -1 * rotateAngle);
        return new Circle(center, radius);
    }

    private static double calculateH (Point currPosRotated, Point finalPosRotated) {
        double x0, y0, x1, y1;
        x0 = currPosRotated.x;
        y0 = currPosRotated.y;
        x1 = finalPosRotated.x;
        y1 = finalPosRotated.y;
        double a = (y1 * y1) - (y0 * y0) - ((x1 - x0) * (x1 - x0));
        double b = 2 * (y0 - y1);
        return a / b;
    }
    private static double calculateK (Point currPosRotated) { return currPosRotated.x; }
    private static double calculateR (double k, Point currPosRotated){ return currPosRotated.x - k; }

}