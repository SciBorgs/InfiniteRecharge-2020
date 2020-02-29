package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.DefaultValue;

public class CircleController {

    private static final double FINAL_HEADING_P = .28;
    private static final double DESIRED_HEADING_P = .4;
    private static final double ENDING_TURN_P = .1;
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0.0); // error for getting our correct heading at the end
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0); // error for getting to the the correct point at the end
    private PID endingTurnPID = new PID(ENDING_TURN_P, 0, 0); // fix heading at close distance

    private static final double ENDING_DISTANCE_TOLERANCE = .1;
    private static final double TURNING_WEIGHT = 0; 
    private static final double STRAIGHT_DISTANCE = ENDING_DISTANCE_TOLERANCE * 2/3;
    private static final double DEFAULT_SPEED = 18;
    private double speed;
    

    private Waypoint destination;

    public CircleController () { this(DEFAULT_SPEED); }

    public CircleController (double speed) { 
        Robot.logger.logFinalPIDConstants("final heading PID", this.finalHeadingPID);
        this.speed = speed;  
    }

    public void update(Waypoint finalDestination) {
        this.destination = finalDestination;
        update(Robot.getPos(), Robot.getHeading(), finalDestination);
    }

    public void update (Point currPos, double currHeading, Waypoint finalDestination) {
        setup();
        Line sightLine = Geo.pointAngleForm(currPos, currHeading);
        if (sightLine.contains(finalDestination.point) || Geo.getDistance(sightLine, currPos) < STRAIGHT_DISTANCE) { // go straight
            //Robot.driveSubsystem.setSpeedTankTurningPercentage(0);
            Robot.driveSubsystem.setSpeedTankForwardTurningPercentage(speed, 0);
        } else {
            setSpeed(currPos, currHeading, finalDestination.point, finalDestination.heading);
        }
        Robot.logger.addData("CircleController.java", "finalHeading", finalDestination.heading, DefaultValue.Empty);
    }

    private static void setup() { Robot.driveSubsystem.assistedDriveMode(); }
    
    private static double getFinalHeadingError(Point finalPos, double finalHeading, Point currPos, double currHeading)   { 
        // get info for circle
        Circle currCircle = Circle.twoPointTangentAngleForm(currPos, currHeading, finalPos);
        double expectedFinalHeading   = Geo.thetaOf(Geo.getTangentToCircle(currCircle, finalPos)); // heading of tangent to circle at final position

        // adjusts heading along the circle
        double currDirection  = Geo.subtractAngles(currHeading,          Geo.angleBetween(currPos,  currCircle.center));
        double finalDirection = Geo.subtractAngles(expectedFinalHeading, Geo.angleBetween(finalPos, currCircle.center));

        // Making sure that we have the correct direction because the creation of the circle
        //  is created the same if you are faced in the opposite direction
        if (Utils.signOf(currDirection) != Utils.signOf(finalDirection)) {
            expectedFinalHeading = Geo.normalizeAngle(expectedFinalHeading + Geo.ANGLE_RANGE / 2);
        }
        double finalHeadingError = Geo.subtractAngles(expectedFinalHeading, finalHeading);
        Robot.logger.addData("CircleController.java", "finalDirection", finalDirection, DefaultValue.Empty);
        Robot.logger.addData("CircleController.java", "expectedFinalHeading", expectedFinalHeading, DefaultValue.Empty);
        Robot.logger.addData("CircleController.java", "finalHeadingError", finalHeadingError, DefaultValue.Empty);
        return finalHeadingError;
    }

    private static double getDesiredHeadingError(Point currPos, double currHeading, Point finalPos) { 
        double desiredHeadingError = Geo.subtractAngles(Geo.angleBetween(currPos, finalPos), currHeading);
        Robot.logger.addData("CircleController.java", "desiredHeadingError", desiredHeadingError, DefaultValue.Empty);
        return desiredHeadingError;
    }

    private void addErrors(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        this.finalHeadingPID  .addMeasurement(getFinalHeadingError  (currPos, currHeading, finalPos, finalHeading));
        this.desiredHeadingPID.addMeasurement(getDesiredHeadingError(currPos, currHeading, finalPos));
    }

    private void setSpeed(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        addErrors(currPos, currHeading, finalPos, finalHeading);
        double turnMagnitude;
        // fix correct heading at close distance
        if (Geo.getDistance(currPos, finalPos) < ENDING_DISTANCE_TOLERANCE) {
            double endingError = Geo.subtractAngles(finalHeading, currHeading);
            this.endingTurnPID.addMeasurement(endingError);
            turnMagnitude = this.endingTurnPID.getOutput();
            Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(0, turnMagnitude);
        } else {
            // want stronger bias towards our heading at the end as we get closer to the point -> weight
            DelayedPrinter.print("Final heaidng pid: " + this.finalHeadingPID.getOutput(), 5);
            turnMagnitude = getWeight(currPos, finalPos) * this.finalHeadingPID.getOutput() + this.desiredHeadingPID.getOutput();
            Robot.driveSubsystem.setSpeedTankForwardTurningPercentage(this.speed, turnMagnitude * (1 - (1 / (1 + (0.7 * Geo.getDistance(currPos, finalPos))))));
            DelayedPrinter.print("turn magin: " + turnMagnitude, 5);
            // Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
        }
        Robot.logger.addData("CircleController.java", "turnMagnitude", turnMagnitude, DefaultValue.Empty);
    }

    private double getWeight(Point currPos, Point finalPos) {
       return 1 + TURNING_WEIGHT * Math.pow(Geo.getDistance(currPos, finalPos), speed);
    }

    public boolean isFinished () {
        return Geo.getDistance(Robot.getPos(), destination.point) < ENDING_DISTANCE_TOLERANCE;
    }
}