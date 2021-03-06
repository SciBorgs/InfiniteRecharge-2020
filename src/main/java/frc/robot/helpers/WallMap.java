package frc.robot.helpers;

import frc.robot.shapes.Line;
import frc.robot.shapes.LineSegment;
import frc.robot.shapes.Ray;
import frc.robot.shapes.Point;

import java.util.ArrayList;

public interface WallMap{

    // Returns an arraylist with all points on the map that intersect the given LineSegment/Ray/Line
    ArrayList<Point> allIntersections(LineSegment ls); 
    ArrayList<Point> allIntersections(Ray r); 
    ArrayList<Point> allIntersections(Line l); 
    
    // Determins wether or not a line segment intersects a place on the map
    boolean intersects(LineSegment ls);
    boolean intersects(Ray r);
    boolean intersects(Line l);
}