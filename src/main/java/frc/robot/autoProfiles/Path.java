package frc.robot.autoProfiles;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashSet;

import frc.robot.shapes.Waypoint;
import frc.robot.dataTypes.Pair;

public class Path {

    private ArrayList<Pair<Waypoint, Boolean>> path;

    public Path (ArrayList<Pair<Waypoint, Boolean>> path) {
        this.path = path;
    }

    
}