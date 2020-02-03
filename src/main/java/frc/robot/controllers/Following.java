package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.logging.LogUpdater;

// Follows an object at a certian angle

public class Following implements LogUpdater {
	private final String FILENAME = "Following.java";

    public final static int lineupSmoother = 5;
    public final static double LINEUP_SHIFT = -.95;

    public Following() {
        Robot.addLogUpdater(this);
    }

    public void followObject(PID pid, double angleTo){
        // Angle to = how much the robot would have to turn to be pointing at the object
        pid.addMeasurement(angleTo);
        double turnMagnitude = pid.getOutput();
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);  
    }

    public void periodicLog(){
    }
}