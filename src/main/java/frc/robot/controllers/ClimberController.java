package frc.robot.controllers;

import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.Robot;
public class ClimberController {
    private double MAXIMUM_TILT_ANGLE = Math.toRadians(14.5);
    private double turnP = 1/(Math.PI/2);
    private PID turnPID;
    private double reachP = 1;
    private PID reachPID;
    private double tiltP = 1/MAXIMUM_TILT_ANGLE;
    private PID tiltPID;
    

    public ClimberController() {
        this.turnPID  = new PID(this.turnP,  0, 0);
        this.reachPID = new PID(this.reachP, 0, 0);
        this.tiltPID  = new PID(this.tiltP,  0, 0);
    }

    public void adjustAngle() { // for better lining up with the bar
        double angleGoal = Math.toRadians(67.5); // turing towards red trench
        double angleCurrent = Robot.get(SD.Angle) % (2 * Math.PI);
        if ( angleCurrent > 7/8 * Math.PI || angleCurrent < 3/8 * Math.PI ) { // if the robot is angled closer to the blue trench then the red trench for grabbing the bar
            angleGoal -= Math.PI; // set the angle goal towards the blue trench
        }
        this.turnPID.addMeasurement(angleGoal - Robot.get(SD.Angle));
        Robot.driveSubsystem.setSpeedTankTurningPercentage(this.turnPID.getOutput());
    }

    public void moveToHeight(double height) {
        this.reachPID.addMeasurement(height-Robot.get(SD.ClimberHeight));
        Robot.climberSubsystem.setLiftSpeed(this.reachPID.getOutput());
    }

    public void moveToBalance() {
        // always going to want to move up the rod
        this.tiltPID.addMeasurement(0 - Robot.get(SD.TiltAngle));
        Robot.climberSubsystem.setShiftMotorSpeed(this.tiltPID.getOutput());
        
    }
}