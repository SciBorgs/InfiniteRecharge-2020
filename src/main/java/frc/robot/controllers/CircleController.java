package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.DefaultValue;

public class CircleController {

    private static final double FINAL_HEADING_P = .4;
    private static final double DESIRED_HEADING_P = .2;
    private static final double ENDING_TURN_P = .1;
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0); // error for getting our correct heading at the end
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, .02); // error for getting to the the correct point at the end
    private PID endingTurnPID = new PID(ENDING_TURN_P, 0, 0); // fix heading at close distance

    private static final double ENDING_DISTANCE_TOLERANCE = .4;
    private static final double TURNING_WEIGHT = .5; // it will be raised to a power of this, ie x^TURNING_WEIGHT

    private double turnMagnitude;

    private final String FILENAME = "CircleController.java";

    public CircleController () { Robot.logger.logFinalPIDConstants("CircleController.java", "final heading PID", this.finalHeadingPID);  }

    public void update(Point finalPos, double finalHeading) {
        Point currPos = Robot.getPos();
        double currHeading = Robot.getHeading();
        setup();
        Line sightLine = Geo.pointAngleForm(currPos, currHeading);
        if (sightLine.contains(finalPos)) { // go straight
            Robot.driveSubsystem.setSpeedTankTurningPercentage(0);
        } else {
            setSpeed (currPos, currHeading, finalPos, finalHeading);
        }
        Robot.logger.addData("CircleController.java", "finalHeading", finalHeading, DefaultValue.Previous);
        // periodicLog();
    }

    private static void setup() { Robot.driveSubsystem.assistedDriveMode(); }
    
    private static double getFinalHeadingError(Point finalPos, double finalHeading, Point currPos, double currHeading)   { 
        // get info for circle
        Circle currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
        double expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos)); // heading of tangent to circle at final position
        double expectedCurrentHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, currPos)); // print for testing

        // adjusts heading along the circle
        double currDirection = Geo.subtractAngles(currHeading, Geo.angleBetween(currPos, currCircle.center));
        double finalDirection = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));

        // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
        if (Utils.signOf(currDirection) != Utils.signOf(finalDirection)) {
            expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
        }
        double finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
        Robot.logger.addData("CircleController.java", "finalDirection", finalDirection, DefaultValue.Previous);
        Robot.logger.addData("CircleController.java", "expectedCurrentHeading", expectedCurrentHeading, DefaultValue.Previous);
        Robot.logger.addData("CircleController.java", "expectedFinalHeading", expectedFinalHeading, DefaultValue.Previous);
        Robot.logger.addData("CircleController.java", "finalHeadingError", finalHeadingError, DefaultValue.Previous);
        return finalHeadingError;
    }

    private static double getDesiredHeadingError(Point currPos, double currHeading, Point finalPos) { 
        double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
        Robot.logger.addData("CircleController.java", "desiredHeadingError", desiredHeadingError, DefaultValue.Previous);
        return desiredHeadingError;
    }

    private void addErrors(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        finalHeadingPID.addMeasurement(getFinalHeadingError(finalPos, finalHeading, currPos, currHeading));
        desiredHeadingPID.addMeasurement(getDesiredHeadingError(currPos, currHeading, finalPos));
    }

    private void setSpeed(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        addErrors(currPos, currHeading, finalPos, finalHeading);
        // fix correct heading at close distance
        if (Geo.getDistance(currPos, finalPos) < ENDING_DISTANCE_TOLERANCE) {
            double endingError = Geo.subtractAngles(finalHeading, currHeading);
            endingTurnPID.addMeasurement(endingError);
            turnMagnitude = endingTurnPID.getOutput();
            Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(0, turnMagnitude);
        } else {
            // want stronger bias towards our heading at the end as we get closer to the point -> weight
            turnMagnitude = getWeight(currPos, finalPos) * finalHeadingPID.getOutput() + desiredHeadingPID.getOutput();
            Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
        }
        Robot.logger.addData("CircleController.java", "turnMagnitude", turnMagnitude, DefaultValue.Previous);
    }

    private static double getWeight(Point currPos, Point finalPos) {
       return 1 + Math.pow(Geo.getDistance(currPos, finalPos), TURNING_WEIGHT);
    }    
}