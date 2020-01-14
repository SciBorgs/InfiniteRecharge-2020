package frc.robot.controllers;

import frc.robot.controllers.PID;
import frc.robot.helpers.Geo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.Robot;
public class ClimberController {
    private double MAXIMUM_TILT_ANGLE = Math.toRadians(14.5);
    //private double turnP = 1/(Math.PI/2);
    //private PID turnPID;
    //private double reachP = 1;
    //private PID reachPID;
    private double tiltP = 1/MAXIMUM_TILT_ANGLE;
    private PID tiltPID;
    

    public ClimberController() {
        //this.turnPID  = new PID(this.turnP,  0, 0);
        //this.reachPID = new PID(this.reachP, 0, 0);
        this.tiltPID  = new PID(this.tiltP,  0, 0);
    }

    /*
    public void moveToHeight(double height) {
        this.reachPID.addMeasurement(height-Robot.get(SD.ClimberHeight));
        Robot.climberSubsystem.setLiftSpeed(this.reachPID.getOutput());
    }
    */

    public void moveToBalance() {
        // always going to want to move up the rod
        this.tiltPID.addMeasurement(0 - Robot.get(SD.TiltAngle));
        Robot.climberSubsystem.setShiftMotorSpeed(this.tiltPID.getOutput());
        
    }
}