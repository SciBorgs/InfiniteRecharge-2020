package frc.robot.autoProfiles;

import java.util.ArrayList;

public class Path {

    public ArrayList<Segment> sequence;
    
    public Path (ArrayList<Segment> path) {
        this.sequence = path;
    }
}