package frc.robot.controllers;

import java.util.ArrayList;
import frc.robot.shapes.*;
import frc.robot.controllers.CircleController;

public class Sequential {
    
    ArrayList<Point> path;
    CircleController SQUAREController;

    public Sequential (ArrayList<Point> path) { 
        this.path = path; 
        SQUAREController = new CircleController();
    }

    public void update () { 
        if (SQUAREController.isFinished) {
            
        }
    }

}