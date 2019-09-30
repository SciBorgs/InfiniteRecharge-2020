package frc.robot.helpers;

import frc.robot.Robot;

public class CircleController { 

    private static final double ROBOT_WIDTH = 0;
    private static final double ANGLE_P = 0;
    private static final double POS_P = 0;
    private static final double TURN_P = 0;
    private PID anglePID = new PID(ANGLE_P, 0, 0);
    private PID posPID = new PID(POS_P, 0, 0);
    private PID turnPID = new PID(TURN_P, 0, 0);

    public CircleController () {}

    public void update (Point currPos, double currHeading, Point finalPos, double finalHeading) {
        Circle currCircle  = Circle.makeCircleWithTangentAnd2Points(currPos, currHeading, finalPos);
        
        double curvature = 1 / currCircle.radius;
        double targetTurnMagnitude = 1 + (ROBOT_WIDTH * curvature) / 2;

        double errorPos   = Geo.getMagnitude(Geo.sub(currPos, finalPos));
        double errorAngle = finalHeading - currHeading;
        anglePID.addMeasurement(errorAngle);
        posPID.addMeasurement(errorPos);
        double turnMagnitude = anglePID.getOutput() + posPID.getOutput();
        
        double errorTurn = targetTurnMagnitude - turnMagnitude;
        turnPID.addMeasurement(errorTurn);
        
        Robot.driveSubsystem.setSpeedTankTurningPercentage(turnPID.getOutput());
    }




}