package frc.robot.helpers;

import frc.robot.Robot;

public class CircleController { 

    private static final double FINAL_HEADING_P = 0;
    private static final double DESIRED_HEADING_P = 0;
    private PID finalHeadingPID   = new PID(FINAL_HEADING_P, 0, 0);
    private PID desiredHeadingPID = new PID(DESIRED_HEADING_P, 0, 0);

    public CircleController () {}

    public void update (Point currPos, double currHeading, Point finalPos, double finalHeading) {
        Circle currCircle  = Circle.makeCircleWithTangentAnd2Points(currPos, currHeading, finalPos);
        Line tangentOnCircleToFinalPos = Geo.getTangentToCircle(currCircle, finalPos);
        double angleOfTangentOnCircleToFinalPos = Geo.thetaOf(tangentOnCircleToFinalPos);

        int directionOnCircle = Geo.getDirectionOnCircle(currCircle, currPos, currHeading); // -1 if cw and 1 if ccw

        double finalHeadingError = finalHeading - angleOfTangentOnCircleToFinalPos;
        finalHeadingPID.addMeasurement(finalHeadingError);

        double desiredHeadingError = finalHeading - currHeading;
        desiredHeadingPID.addMeasurement(desiredHeadingError);

        double turnMagnitude = (desiredHeadingPID.getOutput() + finalHeadingPID.getOutput()) * directionOnCircle;  
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnMagnitude);
    }
}