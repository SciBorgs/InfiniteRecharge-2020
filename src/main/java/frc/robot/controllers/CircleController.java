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

    private Point currPos;
    private Point finalPos;
    private double currHeading;
    private double finalHeading;

    private Circle currCircle;
    private double expectedFinalHeading;
    private double expectedCurrentHeading; // for testing purposes

    private double currDirection;
    private double finalDirection;

    private double finalHeadingError;
    private double desiredHeadingError;

    private double turnMagnitude;

    private final String FILENAME = "CircleController.java";

    public void update(Point finalPos, double finalHeading) {
        setup(finalPos, finalHeading);
        Line sightLine = Geo.pointAngleForm(currPos, currHeading);
        if (sightLine.contains(finalPos)) { // go straight
            Robot.driveSubsystem.setSpeedTankTurningPercentage(0);
        } else {
            fixDirection();
            addErrors();
            setSpeed();
        }
        periodicLog();
    }

    private void setup(Point finalPos, double finalHeading) {
        setup(finalPos, finalHeading, Robot.getPos(), Robot.getHeading());
    }

    private void setup(Point finalPos, double finalHeading, Point currPos, double currHeading) {
        this.finalPos = finalPos;
        this.finalHeading = finalHeading;
        this.currPos = currPos;
        this.currHeading = currHeading;
        Robot.driveSubsystem.assistedDriveMode();

        // get info for circle
        currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
        expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos)); // heading of tangent to circle at final position
        expectedCurrentHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, currPos)); // print for testing
    }

    private void fixDirection() {
        // adjusts heading along the circle
        this.currDirection = Geo.subtractAngles(currHeading, Geo.angleBetween(currPos, currCircle.center));
        this.finalDirection = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));
        // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
        if (Utils.signOf(currDirection) != Utils.signOf(finalDirection)) {
            expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
        }
    }
    
    private double getFinalHeadingError()   { return finalHeadingError  = Geo.subtractAngles(expectedFinalHeading, finalHeading); }
    private double getDesiredHeadingError() { return desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading); }
    private void addErrors() {
        finalHeadingPID.addMeasurement(getFinalHeadingError());
        desiredHeadingPID.addMeasurement(getDesiredHeadingError());
    }

    private void setSpeed() {
        // fix correct heading at close distance
        if (Geo.getDistance(currPos, finalPos) < ENDING_DISTANCE_TOLERANCE) {
            double endingError = Geo.subtractAngles(finalHeading, currHeading);
            endingTurnPID.addMeasurement(endingError);
            turnMagnitude = endingTurnPID.getOutput();
            Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(0, turnMagnitude);
        } else {
            // want stronger bias towards 
            turnMagnitude = getWeight() * finalHeadingPID.getOutput() + desiredHeadingPID.getOutput();
            Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
        }
    }

    private double getWeight() {
       return 1 + Math.pow(Geo.getDistance(currPos, finalPos), TURNING_WEIGHT);
    }    

    public void periodicLog() {
        Robot.logger.logFinalPIDConstants(FILENAME, "final heading PID", this.finalHeadingPID);
        Robot.logger.addData(FILENAME, "expectedFinalHeading", expectedFinalHeading, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "currDirection", currDirection, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "finalDirection", finalDirection, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "finalHeading", finalHeading, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "desiredHeadingError", desiredHeadingError, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "finalHeadingError", finalHeadingError, DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "turnMagnitude", turnMagnitude, DefaultValue.Previous);
    }

}