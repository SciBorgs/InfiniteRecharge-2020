package frc.robot.helpers;

import java.util.Optional;

import frc.robot.routing.navigationmesh.Edge;

public class Geometry {
    public static final Point ORIGIN = new Point(0, 0);
    private static final double EPSILON = 1e-9;

    public static Point rotatePoint(Point point, Point pointToRotateAround, double radiansToRotate) {
        double tempX = point.x - pointToRotateAround.x;
        double tempY = point.y - pointToRotateAround.y;

        return new Point(tempX * Math.cos(radiansToRotate) - tempY * Math.sin(radiansToRotate) + pointToRotateAround.x,
                         tempY * Math.cos(radiansToRotate) + tempX * Math.sin(radiansToRotate) + pointToRotateAround.y);
    }

    public static Line createLine(Point point, double m){return new Line(m, point.y - (m * point.x));}

    public static Optional<Point> getIntersection(Line line1, Line line2) {
        // Return empty value if the lines are parallel
        if (line1.m == line2.m){return Optional.empty();}

        double x = (line2.b - line1.b) / (line1.m - line2.m);
        return Optional.of(new Point(x, line1.m * x + line1.b));
    }

    public static Point getMidpoint(Point point1, Point point2) {
        return new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
    }

    public static Line getPerpendicular(Line line, Point point){return createLine(point, -1 / line.m);}

    public static double getDistance(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point2.x - point1.x, 2) + Math.pow(point2.y - point1.y, 2));
    }

    public static double getDistance(Line line, Point point) {
        Optional<Point> intersection = getIntersection(getPerpendicular(line, point), line);
        // Intersection will never be empty in this case
        return getDistance(point, intersection.get());
    }

    public static boolean arePointsCollinear(Point p1, Point p2, Point p3) {
        return (p2.y - p1.y) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.y - p2.y) <= EPSILON;
    }

    private static double getOrientation(Point point, Edge edge) {
        return (edge.dest.x - edge.origin.x) * (point.y - edge.origin.y) - (point.x - edge.origin.x) * (edge.dest.y - edge.origin.y);
    }

    public static boolean isRightOf(Point point, Edge edge) {
        return getOrientation(point, edge) < 0;
    }

    public static boolean isLeftOf(Point point, Edge edge) {
        return getOrientation(point, edge) > 0;
    }

    public static boolean isPointInCircle(Point p1, Point p2, Point p3, Point pointToTest) {
        double p1dx = p1.x - pointToTest.x;
        double p1dy = p1.y - pointToTest.y;
        double p2dx = p2.x - pointToTest.x;
        double p2dy = p2.y - pointToTest.y;
        double p3dx = p3.x - pointToTest.x;
        double p3dy = p3.y - pointToTest.y;

        double p1p2Det = p1dx * p2dy - p2dx * p1dy;
        double p2p3Det = p2dx * p3dy - p3dx * p2dy;
        double p3p1Det = p3dx * p1dy - p1dx * p3dy;
        
        double p1Lift = Math.pow(p1dx, 2) + Math.pow(p1dy, 2);
        double p2Lift = Math.pow(p2dx, 2) + Math.pow(p2dy, 2);
        double p3Lift = Math.pow(p3dx, 2) + Math.pow(p3dy, 2);

        return p1Lift * p2p3Det + p2Lift * p3p1Det + p3Lift * p1p2Det > 0;
    }
}