package frc.robot.routing;

import java.util.ArrayList;
import java.util.List;

import frc.robot.helpers.Point;
import frc.robot.helpers.WallMap;

public class Router {
    private Point currentPoint;
    private Point goalPoint;
    private WallMap wallMap;
    
    public Router(Point currentPoint, Point goalPoint, WallMap wallMap) {
        this.currentPoint = currentPoint;
        this.goalPoint    = goalPoint;
        this.wallMap      = wallMap;
    }

    public List<Point> computeRoute() {
        return new ArrayList<Point>();
    }
}