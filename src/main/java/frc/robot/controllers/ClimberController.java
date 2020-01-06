package frc.robot.controllers;

import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.Robot;
import frc.robot.PortMap;

public class ClimberController {
    private final double MAXIMUM_TILT_ANGLE = Math.toRadians(14.5);
    private final double reachP = 1;
    private final double tiltP  = 1/MAXIMUM_TILT_ANGLE;
    private double tiltOffset   = 0;

    private PID reachPID;
    private PID tiltPID;

    public ClimberController() {
        reachPID = new PID(reachP, 0, 0);
        tiltPID  = new PID(tiltP, 0, 0);
    }

    public void beforeReach() {
        tiltOffset = Robot.get(SD.TiltAngle);
    }

    public void moveToHeight(double height) {
        reachPID.addMeasurement(height - Robot.get(SD.ClimberHeight));
        Robot.climberSubsystem.setLiftSpeed(reachPID.getOutput());
    }

    /*public void moveToAngle(double angle) {
        tiltPID.addMeasurement(angle - Robot.get(SD.TiltAngle));
        Robot.climberSubsystem.setLiftSpeed(tiltPID.getOutput())
    } No */
}