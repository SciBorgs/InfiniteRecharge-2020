package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.Geo;

public class CircleController { 

    private static final double ERROR = 0.001;
    private static final double FINAL_HEADING_P = 0;
    private static final double DESIRED_HEADING_P = 0;
    private PID finalHeadingPID   = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0);


    public void update (Point currPos, double currHeading, Point finalPos, double finalHeading) {
        Circle currCircle  = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
        double expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos));

        // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90. 
        double angle1 = Geo.normalizeAngle(currHeading)  - Geo.angleBetween(currPos,  currCircle.center);
        double angle2 = Geo.normalizeAngle(finalHeading) - Geo.angleBetween(finalPos, currCircle.center);

        if (Utils.inRange(angle1, angle2, ERROR)) { expectedFinalHeading *= -1; }

        double finalHeadingError = Geo.subtractAngles(finalHeading, expectedFinalHeading);
        finalHeadingPID.addMeasurement(finalHeadingError);

        double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
        desiredHeadingPID.addMeasurement(desiredHeadingError);

        double turnMagnitude = desiredHeadingPID.getOutput() + finalHeadingPID.getOutput();  
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
    }
}