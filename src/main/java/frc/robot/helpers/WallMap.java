package frc.robot.helpers;

import java.util.ArrayList;

public interface WallMap{

    // Returns an arraylist with all points on the map that intersect the given LineSegment
    ArrayList<Point> allIntersections(LineSegment ls); 

    // Determins wether or not a line segment intersects a place on the map
    boolean intersects(LineSegment ls);
}