package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.helpers.Geo;
import frc.robot.robotState.RobotState.SD;
public class AngleController {
    private PID pid;
    private double P = 1/Math.PI;

    public AngleController() {
        this.pid = new PID(this.P, 0, 0);
    }
    
    public void goToAngle(double desiredAngle) {
        desiredAngle = Geo.normalizeAngle(desiredAngle);
        double currentAngle = Geo.normalizeAngle(Robot.get(SD.Angle));
        pid.addMeasurement(Geo.subtractAngles(desiredAngle, currentAngle));
        Robot.driveSubsystem.setSpeedTankTurningPercentage(pid.getOutput());
    }


}