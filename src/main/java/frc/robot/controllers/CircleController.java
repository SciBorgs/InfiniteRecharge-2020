package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.DefaultValue;

public class CircleController {

    private static final double ERROR = 0.02;
    private static final double FINAL_HEADING_P = .8;
    private static final double DESIRED_HEADING_P = .4;
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0);

    public void update(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        Line sightLine = Geo.pointAngleForm(currPos, currHeading);
        if (sightLine.contains(finalPos)) {
            Robot.driveSubsystem.setSpeedTank(.5, .5);
        } else {

            Circle currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
            double expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos));

            // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
            double angle1 = Geo.normalizeAngle(currHeading) - Geo.angleBetween(currPos, currCircle.center);
            double angle2 = Geo.normalizeAngle(expectedFinalHeading) - Geo.angleBetween(finalPos, currCircle.center);

            if (!Utils.inRange(angle1, angle2, ERROR)) {
                // DelayedPrinter.print("negating expected final heading");
                expectedFinalHeading *= -1;
            }

            double finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
            finalHeadingPID.addMeasurement(finalHeadingError);

            double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
            desiredHeadingPID.addMeasurement(desiredHeadingError);
            DelayedPrinter.print("expectedFinalHeading: " + expectedFinalHeading);
            // DelayedPrinter.print("center: (" + currCircle.center.x +", " +
            // currCircle.center.y + ")\t radius: " + currCircle.radius);
            DelayedPrinter.print("finalHeadingError: " + Math.toDegrees(finalHeadingError) + "\tdesiredHeadingError: "
                    + Math.toDegrees(desiredHeadingError));

            double turnMagnitude = desiredHeadingPID.getOutput() + finalHeadingPID.getOutput();
            DelayedPrinter.print("turnMagnitude: " + turnMagnitude);
            Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude * -.2);
            // Robot.driveSubsystem.setSpeedTank((.5 + .2* turnMagnitude)*-1,( .5- .1*
            // turnMagnitude)*-1);
        }
    }
}
