package frc.robot.autoProfiles;

import java.util.ArrayList;
import java.util.List;

import frc.robot.shapes.Waypoint;
import frc.robot.dataTypes.Pair;
import frc.robot.helpers.Geo;
import frc.robot.shapes.Point;

public class AutoRoutine {

    public Path path;
    private final double yShift = 8.21055 / 2;
    private final double xShift = 15.98295 / 2;

    public void tenBallAuto() {
        // start : (3, -7.5), heading: 0
        Waypoint one = new Waypoint(new Point(6.3 - xShift, -7.5 - yShift), Geo.HORIZONTAL_ANGLE);
        Waypoint two = new Waypoint(new Point(4.6 - xShift, -5 - yShift), Geo.HORIZONTAL_ANGLE - Math.PI / 2);
        Waypoint three = new Waypoint(new Point(3.1 - xShift, -2.4 - yShift), Geo.HORIZONTAL_ANGLE);
        Waypoint four = new Waypoint(new Point(8 - xShift, -.7 - yShift), Geo.HORIZONTAL_ANGLE);
        Waypoint five = new Waypoint(new Point(3.1 - xShift, -2.4 - yShift), Geo.HORIZONTAL_ANGLE);
        Pair<Waypoint, Boolean> p1 = new Pair<>(one, true);
        Pair<Waypoint, Boolean> p2 = new Pair<>(two, true);
        Pair<Waypoint, Boolean> p3 = new Pair<>(three, false);
        Pair<Waypoint, Boolean> p4 = new Pair<>(four, true);
        Pair<Waypoint, Boolean> p5 = new Pair<>(five, false);

        ArrayList<Pair<Waypoint, Boolean>> points = new ArrayList<Pair<Waypoint, Boolean>>(List.of(p1, p2, p3, p4, p5));
        this.path = new Path(points);
    }
}