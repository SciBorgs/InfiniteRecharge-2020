package frc.robot.helpers;

import frc.robot.helpers.Vector;

public class Waypoint{
    public Vector position;
    private double curvature, velocity;
    
    public Waypoint(Vector position){
        this.position = position;
    }

    public double getVelocity()  { return velocity;  }
    public double getCurvature() { return curvature; }
    public Vector getPosition()  { return position;  }

	public void setVelocity(double velocity)   { this.velocity  = velocity;  }
    public void setCurvature(double curvature) { this.curvature = curvature; }

}