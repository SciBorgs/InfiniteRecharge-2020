package frc.robot.routing.navigationmesh;

import java.util.HashSet;
import java.util.List;

import frc.robot.helpers.Geometry;
import frc.robot.helpers.Point;
import frc.robot.helpers.Ray;
import frc.robot.helpers.WallMap;

public class Mesh {
    private Point currentPoint;
    private WallMap wallMap;
    private final int ANGLE_INCREMENT = 5;

    public Mesh(Point currentPoint, WallMap wallMap) {
        this.currentPoint = currentPoint;
        this.wallMap = wallMap;
    }

    private HashSet<Point> explore() {
        HashSet<Point> intersections = new HashSet<>();
        Point rayDirection = new Point(this.currentPoint.x, this.currentPoint.y + 1);
        for (int i = 0; i < 360; i += ANGLE_INCREMENT) {
            List<Point> intersectionsAtAngle = this.wallMap.allIntersections(new Ray(this.currentPoint, rayDirection));
            rayDirection = Geometry.rotatePoint(rayDirection, this.currentPoint, Math.toRadians(i));
            for (Point point : intersectionsAtAngle) {
                intersections.add(point);
            }
        }
        return intersections;
    }
}