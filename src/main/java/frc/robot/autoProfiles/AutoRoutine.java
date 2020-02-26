package frc.robot.autoProfiles;

import frc.robot.shapes.Waypoint;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Utils;
import frc.robot.commands.auto.CircleControllerCommand;
import frc.robot.commands.auto.TemporaryInstantCommand;
import frc.robot.commands.drive.ToggleDriveDirection;
import frc.robot.helpers.Geo;

public class AutoRoutine {

    public double yShift = 8.21055 / 2;
    public double xShift = 15.98295 / 2;

    public void tenBallAuto() {
        // start : (3, -7.5), heading: 0       
        Waypoint w1 = new Waypoint(6.3 - xShift, -7.5 - yShift, Geo.HORIZONTAL_ANGLE);
        // s1.doneCommand = new IntakeSuckCommand(2);
        Waypoint w2 = new Waypoint(4.6 - xShift, -5   - yShift, Geo.HORIZONTAL_ANGLE - Math.PI / 2);
        Waypoint w3 = new Waypoint(3.1 - xShift, -2.4 - yShift, Geo.HORIZONTAL_ANGLE);
        // s3.doneCommand = new ShooterCommand();
        Waypoint w4 = new Waypoint(8   - xShift, -.7  - yShift, Geo.HORIZONTAL_ANGLE);
        // s4.doneCommand = new IntakeSuckCommand(2);
        Waypoint w5 = new Waypoint(3.1 - xShift, -2.4 - yShift, Geo.HORIZONTAL_ANGLE);
        // s5.doneCommand = new ShooterCommand();
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.addSequential(new CircleControllerCommand(w2));
        cGroup.addSequential(new CircleControllerCommand(w3));
        cGroup.addSequential(new CircleControllerCommand(w4));
        cGroup.addSequential(new CircleControllerCommand(w5));
    }

    public void test() {
        Waypoint w1 = new Waypoint(3, 1, Geo.HORIZONTAL_ANGLE);
        Waypoint w2 = new Waypoint(6, 0, Geo.HORIZONTAL_ANGLE);
        Waypoint w3 = new Waypoint(9, 1, Geo.HORIZONTAL_ANGLE);
        Waypoint w4 = new Waypoint(12, 0, Geo.HORIZONTAL_ANGLE);
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.addSequential(new CircleControllerCommand(w2));
        cGroup.addSequential(new CircleControllerCommand(w3));
        cGroup.addSequential(new CircleControllerCommand(w4));
        cGroup.start();
    }

    public void test1 () {
        Waypoint w1 = new Waypoint(Utils.inchesToMeters(72), Utils.inchesToMeters(4 * 12), Geo.HORIZONTAL_ANGLE);
        Waypoint w2 = new Waypoint(Utils.inchesToMeters(144), Utils.inchesToMeters(4 * 12), Geo.HORIZONTAL_ANGLE);
        Waypoint w3 = new Waypoint(Utils.inchesToMeters(17 * 12), Utils.inchesToMeters(12), Geo.HORIZONTAL_ANGLE);
        Waypoint w4 = new Waypoint(Utils.inchesToMeters(23 * 12), Utils.inchesToMeters(12), Geo.HORIZONTAL_ANGLE);
        Waypoint w5 = new Waypoint(Utils.inchesToMeters(27 * 12), Utils.inchesToMeters(30), Geo.HORIZONTAL_ANGLE);
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.addSequential(new CircleControllerCommand(w2));
        cGroup.addSequential(new CircleControllerCommand(w3));
        cGroup.addSequential(new CircleControllerCommand(w4));
        cGroup.addSequential(new CircleControllerCommand(w5));       
        cGroup.start();
    }

    public void testDriveDirection() {
        Waypoint w1 = new Waypoint(2.5, 1.0, Geo.HORIZONTAL_ANGLE);
        Waypoint w2 = new Waypoint(0.0, 2.0, Math.PI);
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.addSequential(new ToggleDriveDirection());
        // cGroup.addSequential(new CircleControllerCommand(w2));
        cGroup.start();
    }
}