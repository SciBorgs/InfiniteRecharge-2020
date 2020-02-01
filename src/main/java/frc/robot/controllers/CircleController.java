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
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, .02);
    private PID endingTurnPID = new PID(ENDING_TURN_P, 0, 0);

    private static final double DISTANCE_TOLERANCE = .4;
    private static final double TURNING_WEIGHT = .5; // it will be raised to a power of this, ie x^TURNING_WEIGHT

    private Point currPos;
    private Point finalPos;
    private double currHeading;
    private double finalHeading;
    private Line sightLine;

    private Circle currCircle;
    private double expectedFinalHeading;
    private double expectedCurrentHeading;

    private double angle1;
    private double angle2;

    private double finalHeadingError;
    private double desiredHeadingError;

    private double turnMagnitude;

    private final String FILENAME = "CircleController.java";
    private boolean log = false; // set true to start logging

    public void update(Point finalPos, double finalHeading) {
        setup(finalPos, finalHeading);
        if (sightLine.contains(finalPos)) {
            Robot.driveSubsystem.setSpeedTankTurningPercentage(0);
        } else {
            getPathInfo();
            fixDirection();
            getErrors();
            setSpeed();
        }
        log();
    }

    private void setup(Point finalPos, double finalHeading) {
        this.finalPos = finalPos;
        this.finalHeading = finalHeading;
        currPos = Robot.getPos();
        currHeading = Robot.getHeading();
        sightLine = Geo.pointAngleForm(currPos, currHeading);
        Robot.driveSubsystem.assistedDriveMode();
    }

    private  void getPathInfo() {
        currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
        expectedFinalHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos));
        expectedCurrentHeading = Geo.thetaOf(Geo.getTangentToCircle(currCircle, currPos));
    }

    private void fixDirection() {
        double angle1 = Geo.subtractAngles(currHeading, Geo.angleBetween(currPos, currCircle.center));
        double angle2 = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));
        // We don't need Geo.subtractAngles b/c we expect this to be 90 or -90.
        if (Utils.signOf(angle1) != Utils.signOf(angle2)) {
            expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
        }
    }

    private  void getErrors() {
        finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
        finalHeadingPID.addMeasurement(finalHeadingError);

        desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
        desiredHeadingPID.addMeasurement(desiredHeadingError);
    }

    private void setSpeed(){
        if (Geo.getDistance(currPos, finalPos) < DISTANCE_TOLERANCE) {
            double endingError = Geo.subtractAngles(finalHeading, currHeading);
            endingTurnPID.addMeasurement(endingError);
            turnMagnitude = endingTurnPID.getOutput();
            Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(0, turnMagnitude);
        } else {
            turnMagnitude = desiredHeadingPID.getOutput() + getWeight() * finalHeadingPID.getOutput();
            Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
        }
    }

    private double getWeight(){
       return 1 + Math.pow(Geo.getDistance(currPos, finalPos), TURNING_WEIGHT);
    }    

    public void log() {
        if(log) {
            Robot.logger.logFinalPIDConstants(FILENAME, "final heading PID", this.finalHeadingPID);
            Robot.logger.addData(FILENAME, "expectedFinalHeading", expectedFinalHeading, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "angle1", angle1, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "angle2", angle2, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "finalHeading", finalHeading, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "desiredHeadingError", desiredHeadingError, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "finalHeadingError", finalHeadingError, DefaultValue.Previous);
            Robot.logger.addData(FILENAME, "turnMagnitude", turnMagnitude, DefaultValue.Previous);
        }
    }

}