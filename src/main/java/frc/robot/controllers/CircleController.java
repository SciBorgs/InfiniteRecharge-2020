package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.shapes.*;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.robotState.StateInfo;

public class CircleController {

    private static final double FINAL_HEADING_P = .4;
    private static final double DESIRED_HEADING_P = .2;
    private static final double ENDING_TURN_P = .1;
    public boolean isFinished = false;
    private PID finalHeadingPID = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0);
    private PID endingTurnPID = new PID(ENDING_TURN_P, 0, 0);
    private final String FILENAME = "CircleControllers.java";

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
            double turnMagnitude;
           /* if(Geo.getDistance(currPos, finalPos) < .2) {
                    double endingError = finalHeading - currHeading;
                endingTurnPID.addMeasurement(endingError);
                turnMagnitude = endingTurnPID.getOutput();
                Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(0, turnMagnitude);
                isFinished = true;
            } else*/ if(Geo.getDistance(currPos, finalPos) < .4){
                double endingError = finalHeading - currHeading;
                endingTurnPID.addMeasurement(endingError);
                turnMagnitude = endingTurnPID.getOutput();
                //Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
                Robot.driveSubsystem.setSpeedTankForwardTurningPercentage(.7, turnMagnitude);
            } else {
                turnMagnitude = desiredHeadingPID.getOutput() + finalHeadingPID.getOutput();
                //Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
                Robot.driveSubsystem.setSpeedTankForwardTurningPercentage(.7, turnMagnitude);
                DelayedPrinter.print("turnMagnitude: " + turnMagnitude);
                DelayedPrinter.print("expectedFinalHeading: " + Math.toDegrees(expectedFinalHeading));
                DelayedPrinter.print("expectedCurrentHeading: " + Math.toDegrees(expectedCurrentHeading));
                /*Robot.logger.logFinalPIDConstants(FILENAME, "final heading PID", this.finalHeadingPID);
                Robot.logger.addData(FILENAME, "expectedFinalHeading", expectedFinalHeading, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "angle1", angle1, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "angle2", angle2, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "finalHeading", finalHeading, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "desiredHeadingError", desiredHeadingError, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "finalHeadingError", finalHeadingError, DefaultValue.Previous);
                Robot.logger.addData(FILENAME, "turnMagnitude", turnMagnitude, DefaultValue.Previous);   
                */         
            }
            
            // DelayedPrinter.print("turnMagnitude: " + turnMagnitude);
            // Robot.driveSubsystem.setSpeedTankForwardTurningMagnitude(.3,turnMagnitude);
        }
    }
}