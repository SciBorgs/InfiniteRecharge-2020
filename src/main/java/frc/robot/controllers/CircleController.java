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
        } else {
            Circle currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
            double expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos));
            double expectedCurrentHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, currPos));
            // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
            double angle1 = Geo.subtractAngles(currHeading, Geo.angleBetween(currPos, currCircle.center));
            double angle2 = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));

            if (Utils.signOf(angle1) != Utils.signOf(angle2)) {
                expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
            }

            double finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
            finalHeadingPID.addMeasurement(finalHeadingError);

            double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
            desiredHeadingPID.addMeasurement(desiredHeadingError);

            double turnMagnitude = desiredHeadingPID.getOutput() + finalHeadingPID.getOutput();
            Robot.driveSubsystem.setSpeedTankTurningPercentage(.3*turnMagnitude);
        }
    }
}
