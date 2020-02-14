package frc.robot.shapes;

import java.util.Objects;
import frc.robot.shapes.Point;

public class Waypoint {
    
    public Point point;
    public double heading;

    public Waypoint (Point point, double heading) {
        this.point   = new Point(point.x, point.y);
        this.heading = heading;
        hashCode();
    }

    public String toString () {
        return "Point: " + this.point + "\t Heading: " + this.heading;
    }

    public boolean equals (Waypoint waypoint) { 
        return this.point.equals(waypoint.point) && this.heading == waypoint.heading;
    }

    @Override
    public int hashCode() {
        return Objects.hash(this.point, this.heading);
    }
}