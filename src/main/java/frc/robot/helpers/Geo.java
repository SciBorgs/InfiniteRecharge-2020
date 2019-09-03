package frc.robot.helpers;

import java.util.Optional;

public class Geo {

    public static final Point ORIGIN = new Point(0, 0);
    private static final double EPSILON = 1e-9;
    private static final double DELTA = 1;
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 2 * Math.PI;

    public static Point shiftPoint(Point p, double x, double y) {
        return new Point(p.x + x, p.y + y);
    }

    public static Point rotatePoint(Point point, double theta, Point rotateAround) {
        Point shifted = sub(point, rotateAround);
        Point rotated = rotatePoint(shifted, theta);
        return add(rotated, rotateAround);
    }

    public static Point rotatePoint(Point p, double theta) {
        return new Point(p.x * Math.cos(theta) - p.y * Math.sin(theta), p.y * Math.cos(theta) + p.x * Math.sin(theta));
    }

    public static Point flipXandY(Point p) {
        return new Point(p.y, p.x);
    }

    public static Line flipXandY(Line l) {
        return new Line(flipXandY(l.p1), flipXandY(l.p2));
    }

    public static double thetaOf(LineLike lLike) {
        Line l = lLike.toLine();
        return Math.atan2(l.p1.y - l.p2.y, l.p1.x - l.p2.x);
    }

    public static double mOf(LineLike lLike) { // Slope
        return Math.tan(thetaOf(lLike));
    }

    public static double bOf(LineLike lLike) { // Y-intercept
        Line l = lLike.toLine();
        return l.p1.y - mOf(l) * l.p1.x;
    }

    public static boolean isVertical(LineLike lLike) {
        return thetaOf(lLike) == Math.atan2(DELTA, 0);
    }

    public static double yOf(Line l, double x) {
        return mOf(l) * x + bOf(l);
    }

    public static double xOf(Line l, double y) {
        return yOf(flipXandY(l), y);
    }

    public static Line pointSlopeForm(Point point, double m) {
        return new Line(point, new Point(point.x + DELTA, point.y + DELTA * m));
    }

    public static Line slopeInterceptForm(double m, double b) {
        return pointSlopeForm(new Point(0, b), m);
    }

    public static Line pointAngleForm(Point point, double theta) {
        if (theta == Math.atan2(DELTA, 0)) {
            return new Line(point, new Point(point.x, point.y + DELTA));
        }
        return pointSlopeForm(point, Math.tan(theta));
    }

    public static double getDistance(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point2.x - point1.x, 2) + Math.pow(point2.y - point1.y, 2));
    }

    public static double getDistance(LineLike lLike, Point point) {
        Line fakeLine = lLike.toLine();  
        Optional<Point> intersection = getIntersection(getPerpendicular(fakeLine, point), fakeLine); // Intersection will never be empty in this case
        
        if (!lLike.contains(intersection.get())) {
            double distanceToP1 = getDistance(lLike.p1, point);
            double distanceToP2 = getDistance(lLike.p2, point);

            if (distanceToP1 < distanceToP2) {
                return distanceToP1;
            } else {
                return distanceToP2;
            }
        }
        
        return getDistance(point, intersection.get());
    }

    public static boolean arePointsCollinear(Point p1, Point p2, Point p3) {
        return (p2.y - p1.y) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.y - p2.y) <= EPSILON;
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

    public static Point getMidpoint(Point point1, Point point2) {
        return new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
    }

    public static Line getPerpendicular(Line line, Point point) {
        return pointAngleForm(point, thetaOf(line) + (MAX_ANGLE - MIN_ANGLE) / 4);
    }

    public static double getDirectionOfRay(Ray ray) {
        // Based on the left and right of pEnd
        if (ray.p1.x < ray.p2.x) {
            return 1; // Right
        } else {
            return -1; // Left
        }
    }

    public static boolean isPointOn(AlmostLine almostLine, Point point) {
        return almostLine.contains(point);
    }

    public static boolean areParellel(LineLike l1, LineLike l2) {
        return thetaOf(l1) == thetaOf(l2);
    }

    public static Optional<Point> getIntersection(LineLike lLike1, LineLike lLike2) {
        if (areParellel(lLike1, lLike2)) {
            return Optional.empty();
        }

        // Algorithim: http://geomalgorithms.com/a05-_intersect-1.html
        Point u = sub(lLike1.p2, lLike1.p1);
        Point v = sub(lLike2.p2, lLike2.p1);
        Point w = sub(lLike1.p1, lLike2.p1);

        double s = (v.y * w.x - v.x * w.y) / (v.x * u.y - v.y * u.x);
        Point intersection = add(lLike1.p1, scale(u, s));

        if (lLike1.contains(intersection) && lLike2.contains(intersection)) {
            return Optional.of(intersection);
        } else {
            return Optional.empty();
        }
    }

    // for vectors
    public static Point scale(Point point, double c) {
        return new Point(point.x * c, point.y * c);
    }

    public static Point add(Point A, Point B) {
        return new Point(A.x + B.x, A.y + B.y);
    }

    public static Point sub(Point A, Point B) {
        return add(A, scale(B, -1));
    }

    public static double magnitude(Point A) {
        return Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2));
    }

    public static double dot(Point A, Point B) {
        return A.x * B.x + A.y * B.y;
    }

    public static double angleBetween(Point A, Point B) {
        double dot = dot(A, B);
        double magnitudeA = Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2));
        double magnitudeB = Math.sqrt(Math.pow(B.x, 2) + Math.pow(B.y, 2));
        return Math.acos(dot / (magnitudeA * magnitudeB));
    }
}