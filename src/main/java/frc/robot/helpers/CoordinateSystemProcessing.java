package frc.robot.helpers;

import java.util.Optional;

public class CoordinateSystemProcessing {

    public static final Point ORIGIN = new Point(0, 0);

    public static Point rotatePoint(Point point, Point pointToRotateAround, double radiansToRotate) {
        double tempX = point.x - pointToRotateAround.x;
        double tempY = point.y - pointToRotateAround.y;

        return new Point(tempX * Math.cos(radiansToRotate) - tempY * Math.sin(radiansToRotate) + pointToRotateAround.x,
                         tempY * Math.cos(radiansToRotate) + tempX * Math.sin(radiansToRotate) + pointToRotateAround.y);
    }

    public static Line createLine(Point point, double m){return new Line(m, point.y - (m * point.x));}

    public static Line createLine(Point point1, Point point2) {
        return createLine(point1, (point2.y - point1.y) / (point2.x - point1.x));
    }

    public static double getDistance(Point point1, Point point2) {
        return Math.sqrt(Math.pow(point2.x - point1.x, 2) + Math.pow(point2.y - point1.y, 2));
    }

    public static double getDistance(Line line, Point point) {
        Optional<Point> intersection = getIntersection(getPerpendicular(line, point), line);
        // Intersection will never be empty in this case
        return getDistance(point, intersection.get());
    }

    public static Point getMidpoint(Point point1, Point point2) {
        return new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
    }

    public static Line getPerpendicular(Line line, Point point){return createLine(point, -1 / line.m);}

    public static Optional<Point> getIntersection(Line line1, Line line2) {
        // Return empty value if the lines are parallel
        if (line1.m == line2.m){return Optional.empty();}

        double x = (line2.b - line1.b) / (line1.m - line2.m);
        return Optional.of(new Point(x, line1.m * x + line1.b));
    }

    public static Point scale(Point point, double c) { return new Point(point.x * c, point.y *c); }

    public static double magnitude(Point A) { return Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2)); }
    public static double dot(Point A, Point B) { return A.x * B.x + A.y * B.y; }
    
    public static Point add(Point A, Point B) { return new Point(A.x + B.x, A.y +B.y); }
	public static Point sub(Point A, Point B) { return add(A, scale(B, -1)); }

    public static double angleBetween(Point A, Point B) {
		double dot = dot(A, B);
		double magnitudeA = Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2));
		double magnitudeB = Math.sqrt(Math.pow(B.x, 2) + Math.pow(B.y, 2));
		return Math.acos(dot / (magnitudeA * magnitudeB));
    }
}