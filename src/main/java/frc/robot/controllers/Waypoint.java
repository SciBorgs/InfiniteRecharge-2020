package frc.robot.controllers;

import frc.robot.shapes.Point;

public class Waypoint {
    
    public Point point;
    public double heading;

    public Waypoint(Point point, double heading) {
        this.point   = new Point(point.x, point.y);
        this.heading = heading;
    }
}