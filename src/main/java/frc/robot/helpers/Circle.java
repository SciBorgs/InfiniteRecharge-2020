package frc.robot.helpers;

public class Circle {

    Point  center;
    double radius;

    public Circle(Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Point  getCenter() { return this.center; }
    public double getRadius() { return this.radius; }

    public static Circle makeCircleTangentAnd2Points(Point currPos, double currHeading, Point finalPos) {
        double x0, y0, x1, y1, m, h , k, radius;
        x0 = currPos.x;
        y0 = currPos.y;
        x1 = finalPos.x;
        y1 = finalPos.y;
        m  = Math.atan(currHeading);
        h  = calculateH(x0, y0, x1, y1, m);
        k  = calculateK(x0, y0, m, h);
        radius = calculateR(x0, y0, m, h, k);
        Point center = new Point(h, k);
        return new Circle(center, radius);
    }
    
    public static double calculateH(double x0, double y0, double x1, double y1, double m) {
        double a = (.5 * y1 * y1 - y0 * y0 + x1 * x1 - x0 * x0);
        double b = m * y0 * x1 - m * y0 * x0 + x0 * x1 - x0 * x0;
        double c = m * x0 - m * x1 - y0 + y1;
        return (a - b) / c;
    }
    
    private static double calculateK(double x0, double y0, double m, double h) { return m * (y0 - h) + x0; }
    private static double calculateR(double x0, double y0, double m, double h, double k) { return Math.sqrt(Math.pow(x0 - k, 2) + Math.pow(y0 - h, 2)); }
}