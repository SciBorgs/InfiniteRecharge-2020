package frc.robot.autoProfiles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.generalCommands.EmptyCommand;
import frc.robot.shapes.Point;
import frc.robot.shapes.Waypoint;

public class Segment {
    public Waypoint waypoint;
    public Command  sequentialCommand, parallelCommand, doneCommand;
    public boolean  reverse;
    public WaitCommand startWaitCommand, endWaitCommand;
    public ArrayList<Command> commands;

    public Segment (Waypoint waypoint) {
        this.waypoint = waypoint;
        this.reverse = false;
        this.sequentialCommand = new EmptyCommand();   
        this.parallelCommand = new EmptyCommand();
        this.doneCommand = new EmptyCommand();
        this.startWaitCommand = new WaitCommand(0);
        this.endWaitCommand = new WaitCommand(0);
        commands = new ArrayList<>(Arrays.asList(startWaitCommand, sequentialCommand, parallelCommand, doneCommand, endWaitCommand));
    }

    public Segment (Point point, double heading){
        this(new Waypoint(point, heading));
    }
}