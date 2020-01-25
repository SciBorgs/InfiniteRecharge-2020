package frc.robot.controllers;

import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.controllers.CircleController;
import frc.robot.shapes.Point;

public class Sequential {
    
    ArrayList<Waypoint> path;
    CircleController circleController;
    private int lastPointHitIndex = 0;
    private int currDestinationIndex = 1;
    private Point currDestination;
    private double currDestinationHeading;
    private boolean isFinished;

    public Sequential (ArrayList<Waypoint> path) { 
        this.path = path; 
        circleController = new CircleController();
    }

    public void update () { 
        
        if (!isFinished) {
            currDestination = path.get(currDestinationIndex).point;
            currDestinationHeading = path.get(currDestinationIndex).heading;
            circleController.update(Robot.getPos(),Robot.getHeading(), currDestination,currDestinationHeading);
        }
    }

}