package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.DefaultValue;

public class CircleController {

    private static final double ERROR = 0.02;
    private static final double FINAL_HEADING_P = .4;
    private static final double DESIRED_HEADING_P = .2;
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0);

    public void update(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        Robot.driveSubsystem.assistedDriveMode();
        Line sightLine = Geo.pointAngleForm(currPos, currHeading);
        if (sightLine.contains(finalPos)) {
            Robot.driveSubsystem.setSpeedTankTurningPercentage(0);
            // DelayedPrinter.print("sameLine");
        } else {
            Circle currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
            double expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos));
            double expectedCurrentHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, currPos));
            // DelayedPrinter.print("expectedCurrentHeading: " + expectedCurrentHeading);
            // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
            double angle1 = Geo.subtractAngles(currHeading, Geo.angleBetween(currPos, currCircle.center));
            double angle2 = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));
            // DelayedPrinter.print("angle1: " + Math.toDegrees(angle1));
            // DelayedPrinter.print("angle2: " + Math.toDegrees(angle2));

            if (Utils.signOf(angle1) != Utils.signOf(angle2)) {
                // DelayedPrinter.print("negating expected final heading");
                expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
            }

            double finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
            finalHeadingPID.addMeasurement(finalHeadingError);

            double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
            desiredHeadingPID.addMeasurement(desiredHeadingError);
            // DelayedPrinter.print("expectedFinalHeading: " + Math.toDegrees(expectedFinalHeading));
            // DelayedPrinter.print("finalHeadingError: " + Math.toDegrees(finalHeadingError) + "\tdesiredHeadingError: "
                    // + Math.toDegrees(desiredHeadingError));

            double turnMagnitude = desiredHeadingPID.getOutput() + finalHeadingPID.getOutput();
            // DelayedPrinter.print("turnMagnitude: " + turnMagnitude);
            // Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
            Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(.3,turnMagnitude);
        }
    }
}
