package frc.robot.autoProfiles;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.generalCommands.EmptyCommand;
import frc.robot.shapes.Point;
import frc.robot.shapes.Waypoint;

public class Segment {
    public Waypoint waypoint;
    public Command  sequentialCommand, parallelCommand, doneCommand;
    public boolean  reverse;
    public double   startWait, endWait;

    public Segment () {}

    public Segment (Waypoint waypoint){
        this.waypoint = waypoint;
        this.startWait = 0;
        this.endWait = 0;
        this.reverse = false;
        sequentialCommand =  new EmptyCommand();
        parallelCommand = new EmptyCommand();
        doneCommand = new EmptyCommand();
    }

    public Segment (Point point, double heading){
        this(new Waypoint(point, heading));
    }
}