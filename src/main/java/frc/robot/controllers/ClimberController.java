package frc.robot.controllers;

import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.Robot;
public class ClimberController {
    private double MAXIMUM_TILT_ANGLE = Math.toRadians(14.5);
    private double reachP = 1;
    private PID reachPID;
    private double tiltP = 1/MAXIMUM_TILT_ANGLE;
    private PID tiltPID;
    private double tiltOffset = 0;
    public ClimberController(){
        reachPID = new PID(reachP, 0, 0);
        tiltPID = new PID(tiltP, 0, 0);
        
    }

    public void beforeReach(){
        tiltOffset = Robot.get(SD.TiltAngle);
    }

    public void moveToHeight(double height){
        reachPID.addMeasurement(height-Robot.get(SD.ClimberHeight));
        Robot.climberSubsystem.setLiftSpeed(reachPID.getOutput());
    }
}