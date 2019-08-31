package frc.robot.helpers;

import java.util.Optional;

public class CoordinateSystemProcessing {

    public static final Point ORIGIN = new Point(0, 0);
    private static final double EPSILON = 1e-9;

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

    public static double getDistance(LineSegment lineSegment, Point point) {
        Point closestPoint;
        double dX = lineSegment.p2.x - lineSegment.p1.x;
        double dY = lineSegment.p2.y - lineSegment.p1.y;

        if (dX == 0 && dY == 0) {
            closestPoint = lineSegment.p1;
            dX = point.x - lineSegment.p1.x;
            dY = point.y - lineSegment.p1.y;
            
            return Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
        } // p1 and p2 cannot be on the same point

        double t = ((point.x - lineSegment.p1.x) * dX + (point.y - lineSegment.p1.y) * dY) / (Math.pow(dX, 2) + Math.pow(dY, 2));

        if (t < 0) {
            closestPoint = new Point(lineSegment.p1.x, lineSegment.p1.y);
            dX = point.x - lineSegment.p1.x;
            dY = point.y - lineSegment.p1.y;
        } else if (t > 1) {
            closestPoint = new Point(lineSegment.p2.x, lineSegment.p2.y);
            dX = point.x - lineSegment.p2.x;
            dY = point.y - lineSegment.p2.y;
        } else {
            closestPoint = new Point(lineSegment.p1.x + t * dX, lineSegment.p1.y + t * dY);
            dX = point.x - closestPoint.x;
            dY = point.y - closestPoint.y;
        }

        return Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
    }

    public static double getDistance(Ray ray, Point point) {
        Line fakeLine = createLine(ray.pEnd, ray.pDir);

        Optional<Point> intersection = getIntersection(getPerpendicular(fakeLine, point), fakeLine);

        Ray fakeRay = new Ray(point, intersection.get());

        double endPointToIntersection;
        double pointToIntersection = getDistance(point, intersection.get());

        if (!(getIntersection(ray, fakeRay).isPresent())) {
            endPointToIntersection = getDistance(fakeLine, point);
        } else {
            endPointToIntersection = getDistance(ray.pEnd, intersection.get());
        }

        return Math.sqrt(Math.pow(endPointToIntersection, 2) + Math.pow(pointToIntersection, 2));
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
    
    public static Line getPerpendicular(Line line, Point point){return createLine(point, -1 / line.m);}

    public static double getDirectionOfRay(Ray ray) {
        // Based on the left and right of pEnd
        if (ray.pEnd.x < ray.pDir.x) {
            return 1; // Right
        } else {
            return -1; // Left
        }
    }

    public static boolean isPointOnLine(Line line, Point point) {
        return point.y == (line.m * point.x) + line.b;
    }
    
    public static boolean isPointOnSegment(LineSegment lineSegment, Point point) {
        return arePointsCollinear(lineSegment.p1, lineSegment.p2, point);
    }
    
    public static boolean isPointOnRay(Ray ray, Point point) {
        double sideOfLine = getDirectionOfRay(ray);
        Line fakeLine = createLine(ray.pEnd, ray.pDir);
    
        if (getDirectionOfRay(new Ray(ray.pEnd, point)) == sideOfLine) {
            if (isPointOnLine(fakeLine, point)) {
                return true;
            }
        }
        return false;
    }

    public static Optional<Point> getIntersection(Line line1, Line line2) { 
        // Return empty value if the lines are parallel
        if (line1.m == line2.m){return Optional.empty();}

        double x = (line2.b - line1.b) / (line1.m - line2.m);
        return Optional.of(new Point(x, line1.m * x + line1.b));
    }
    
    public static Optional<Point> getIntersection(LineSegment lineSegment1, LineSegment lineSegment2) {
        Line fakeLine1 = createLine(lineSegment1.p1, lineSegment1.p2);
        Line fakeLine2 = createLine(lineSegment2.p1, lineSegment2.p2);

        Optional<Point> pointOfIntersection = getIntersection(fakeLine1, fakeLine2);

        if (!isPointOnSegment(lineSegment1, pointOfIntersection.get()) || !isPointOnSegment(lineSegment2, pointOfIntersection.get())) {
            return Optional.empty();
        }

        return pointOfIntersection;
    }

    public static Optional<Point> getIntersection(Ray ray1, Ray ray2) {
        double ray1Direction = getDirectionOfRay(ray1);
        double ray2Direction = getDirectionOfRay(ray2);

        Line fakeLine1 = createLine(ray1.pEnd, ray1.pDir);
        Line fakeLine2 = createLine(ray2.pEnd, ray2.pDir);

        Optional<Point> intersection = getIntersection(fakeLine1, fakeLine2);

        Ray fakeRay1 = new Ray(ray1.pEnd, intersection.get());
        Ray fakeRay2 = new Ray(ray2.pEnd, intersection.get());

        if (!(getDirectionOfRay(fakeRay1) == ray1Direction) && !(getDirectionOfRay(fakeRay2) == ray2Direction)){return Optional.empty();}
        
        return intersection;
    }

    // for vectors
    public static Point scale(Point point, double c) { return new Point(point.x * c, point.y * c); }
    public static Point add(Point A, Point B) { return new Point(A.x + B.x, A.y +B.y); }
	public static Point sub(Point A, Point B) { return add(A, scale(B, -1)); }
    
    public static double magnitude(Point A) { return Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2)); }
    public static double dot(Point A, Point B) { return A.x * B.x + A.y * B.y; }
    public static double angleBetween(Point A, Point B) {
		double dot = dot(A, B);
		double magnitudeA = Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2));
		double magnitudeB = Math.sqrt(Math.pow(B.x, 2) + Math.pow(B.y, 2));
		return Math.acos(dot / (magnitudeA * magnitudeB));
    }
}
