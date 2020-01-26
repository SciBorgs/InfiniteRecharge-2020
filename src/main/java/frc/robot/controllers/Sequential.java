package frc.robot.controllers;

import java.util.ArrayList;

import frc.robot.controllers.CircleController;
import frc.robot.helpers.Geo;
import frc.robot.shapes.Point;

public class Sequential {
    
    private ArrayList<Waypoint> path;
    private CircleController circleController;
    private int lastPointHitIndex = 0;
    private int currDestinationIndex = 1;
    private Point currDestination;
    private double currDestinationHeading;
    private double distanceTolerance = .1;

    public Sequential (ArrayList<Waypoint> path) { 
        this.path = path; 
        circleController = new CircleController();
    }

    public void update (Waypoint currPos) { 
        lastPointHitIndex      = getLastPointHitIndex(currPos.point);
        currDestinationIndex   = lastPointHitIndex + 1;
        currDestination        = path.get(currDestinationIndex).point;
        currDestinationHeading = path.get(currDestinationIndex).heading;
        if (!isFinished()) {
            circleController.update(currPos.point, currPos.heading, currDestination, currDestinationHeading);
        }
    }

    private int getLastPointHitIndex (Point currPos) {
        if (Geo.getDistance(currPos, path.get(lastPointHitIndex + 1).point) < distanceTolerance) {
            lastPointHitIndex++;
            currDestinationIndex++;
        }
        return lastPointHitIndex;
    }

    private boolean isFinished () {
        return lastPointHitIndex == path.size();
    }
}