package frc.robot.helpers;

import frc.robot.Robot;

public class Following {
	private final String FILENAME = "Following.java";

    public final static int lineupSmoother = 5;
    public final static double LINEUP_SHIFT = -.95;

    public Following() {
    }

    public void followObject(PID pid, double tx){
        if (Robot.limelightSubsystem.contourExists()){
            pid.addMeasurement(tx);
        }
        double turnMagnitude = pid.getOutput();
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);  
    }

    public void periodicLog(){
    }
}