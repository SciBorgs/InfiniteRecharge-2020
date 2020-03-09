package frc.robot.autoProfiles;

import frc.robot.shapes.Waypoint;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Utils;
import frc.robot.commands.auto.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.helpers.Geo;

public class AutoRoutine {

    public double yShift = 8.21055 / 2;
    public double xShift = 15.98295 / 2;

    public void eightBallAuto() {
        // start : (3 - xShift, -7.5 - yShift), heading: 0       
        Waypoint trench     = new Waypoint(6.3 - xShift, -7.5 - yShift, Geo.HORIZONTAL_ANGLE);
        double speedToTrench = 15;
        Waypoint midToShoot = new Waypoint(4.6 - xShift, -5   - yShift, Geo.HORIZONTAL_ANGLE + Math.PI / 2);
        Waypoint shootPos   = new Waypoint(3.1 - xShift, -2.4 - yShift, Geo.HORIZONTAL_ANGLE + Math.PI);
        double speedOnPath  = 15;
        Waypoint trench2    = new Waypoint(8   - xShift, -.7  - yShift, Geo.HORIZONTAL_ANGLE + Math.PI); // can be tenBallAuto if the bot goes farther
        CommandGroup cGroup = new CommandGroup();
        cGroup.addParallel  (new IntakeSuckCommand());
        cGroup.addSequential(new CircleControllerCommand(trench, speedToTrench));
        cGroup.addSequential(new ToggleDriveDirection());
        cGroup.addParallel  (new IntakeStopCommand());
        cGroup.addSequential(new CircleControllerCommand(midToShoot, speedOnPath));
        cGroup.addSequential(new CircleControllerCommand(shootPos, speedOnPath));
        cGroup.addSequential(new ShootCommand());
        cGroup.addSequential(new WaitCommand(4)); 
        cGroup.addSequential(new ToggleDriveDirection());
        cGroup.addParallel  (new IntakeSuckCommand());
        cGroup.addSequential(new CircleControllerCommand(trench2, speedToTrench));
        cGroup.addSequential(new ToggleDriveDirection());
        cGroup.addParallel  (new IntakeStopCommand());
        cGroup.addSequential(new CircleControllerCommand(shootPos, speedOnPath));
        cGroup.addSequential(new ShootCommand());
        cGroup.addSequential(new WaitCommand(4)); 
        cGroup.start();
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
        Waypoint w1 = new Waypoint(4.0, 0.0, Geo.HORIZONTAL_ANGLE);
        Waypoint w2 = new Waypoint(2.0, 0.0, Math.PI);
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.addSequential(new ToggleDriveDirection());
        cGroup.addSequential(new CircleControllerCommand(w2));
        cGroup.start();
    }

    // gay
    public void straight (double distance) {
        Waypoint w1 = new Waypoint(distance, 0.0, Geo.HORIZONTAL_ANGLE);
        CommandGroup cGroup = new CommandGroup();
        cGroup.addSequential(new CircleControllerCommand(w1));
        cGroup.start();
    }
}