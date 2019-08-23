package frc.robot.helpers;

import frc.robot.helpers.Point;

public class Waypoint{
    public Point position;
    private double curvature, velocity;
    
    public Waypoint(Point position){ this.position = position; }
    
    public Waypoint(double x, double y){
        this.position.x = x;
        this.position.y = y;
    }

    public double getVelocity()  { return this.velocity;  }
    public double getCurvature() { return this.curvature; }
    public Point getPosition()  { return this.position;  }

	public void setVelocity(double velocity)   { this.velocity  = velocity;  }
    public void setCurvature(double curvature) { this.curvature = curvature; }

}