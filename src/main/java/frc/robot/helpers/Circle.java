package frc.robot.helpers;

public class Circle {
    
    Point  center;
    double radius;

    public Circle (Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public static Circle twoPointTangentAngleForm (Point currPos, double currHeading, Point finalPos) {
        // rotate currPos so that currHeading becomes 0
        Point currPosRotated  = Geo.rotatePoint(currPos,  -1 * currHeading);
        Point finalPosRotated = Geo.rotatePoint(finalPos, -1 * currHeading);

        double kRotated = calculateK(currPosRotated, finalPosRotated);
        double hRotated = currPosRotated.x;
        double radius   = Math.abs(currPosRotated.y - kRotated);

        Point center = Geo.rotatePoint(new Point(hRotated, kRotated), currHeading);
        return new Circle(center, radius);
    }

    private static double calculateK (Point currPosRotated, Point finalPosRotated) {
        double x0, y0, x1, y1;
        x0 = currPosRotated.x;
        y0 = currPosRotated.y;
        x1 = finalPosRotated.x;
        y1 = finalPosRotated.y;
        double a = (y1 * y1) - (y0 * y0) + ((x1 - x0) * (x1 - x0));
        double b = 2 * (y1 - y0);
        return a / b;
    }
}