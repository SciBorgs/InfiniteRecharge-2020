package frc.robot.autoProfiles;

import java.util.ArrayList;
import java.util.List;

import frc.robot.shapes.Waypoint;
import frc.robot.commands.intake.IntakeSuckCommand;
import frc.robot.commands.auto.PathCommand;
import frc.robot.helpers.Geo;
import frc.robot.shapes.Point;

public class AutoRoutine {

    public Path path;
    public double yShift = 8.21055 / 2;
    public double xShift = 15.98295 / 2;

    public void tenBallAuto() {
        // start : (3, -7.5), heading: 0       
        Segment s1 = new Segment(new Waypoint(6.3 - xShift, -7.5 - yShift, Geo.HORIZONTAL_ANGLE));
        // s1.doneCommand = new IntakeSuckCommand(2);
        Segment s2 = new Segment(new Waypoint(4.6 - xShift, -5   - yShift, Geo.HORIZONTAL_ANGLE - Math.PI / 2));
        s2.reverse = true;
        Segment s3 = new Segment(new Waypoint(3.1 - xShift, -2.4 - yShift, Geo.HORIZONTAL_ANGLE));
        s3.reverse = true;
        // s3.doneCommand = new ShooterCommand();
        Segment s4 = new Segment(new Waypoint(8   - xShift, -.7  - yShift, Geo.HORIZONTAL_ANGLE));
        s4.reverse = false;
        // s4.doneCommand = new IntakeSuckCommand(2);
        Segment s5 = new Segment(new Waypoint(3.1 - xShift, -2.4 - yShift, Geo.HORIZONTAL_ANGLE));
        // s5.doneCommand = new ShooterCommand();
        this.path = new Path(new ArrayList<>(List.of(s1, s2, s3, s4, s5)));
        (new PathCommand(path)).start();
    }
}