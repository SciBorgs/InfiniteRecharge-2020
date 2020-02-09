package frc.robot.stateEstimation.explicit;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.shapes.Point;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import frc.robot.stateEstimation.interfaces.*;
import java.util.Hashtable;

public class LimelightLocalization implements MaybeUpdater {
    public LimelightSubsystem limeLight;

    private static final double CAMERA_ABOVE_GROUND_HEIGHT = Utils.inchesToMeters(29);
    private static final double CAMERA_MOUNTING_ANGLE      = Math.toRadians(0);
    private static final double OUTER_PORT_HEIGHT          = Utils.inchesToMeters(90);

    private static final double LANDMARK_X = 0;
    private static final double LANDMARK_Y = 0;

    private Hashtable<SD, Double> stdDevs;

    public LimelightLocalization() {
        this.limeLight = Robot.limelightSubsystem;
        this.stdDevs = new Hashtable<>();
        this.stdDevs.put(SD.X,     0.0);
        this.stdDevs.put(SD.Y,     0.0);
    }

    public double calculateDistance() {
        double yAngle = Math.toRadians(limeLight.getTableData(limeLight.getCameraTable(), "ty"));
        return (OUTER_PORT_HEIGHT - CAMERA_ABOVE_GROUND_HEIGHT)/Math.tan(yAngle + CAMERA_MOUNTING_ANGLE); 
    }

    public void test() {
        //Point pos = getRobotPosition();
        double distance = calculateDistance();
        Point pos = Robot.getPos();
        Robot.limelightSubsystem.setCameraParams("X", pos.x);
        Robot.limelightSubsystem.setCameraParams("Y", pos.y);
        Robot.limelightSubsystem.setCameraParams("Distance", distance);
    }

    public Point getRobotPosition() {
        double distance = calculateDistance();
        double robotAngle = Robot.get(SD.Angle);
        double tx = limeLight.getTableData(limeLight.getCameraTable(), "tx");
        double complement = 90 - (tx + robotAngle);
        double changeInY = distance * Math.cos(Math.toRadians(complement));
        double changeInX = Math.sqrt(distance * distance - changeInY * changeInY);
        double yAbsolute = LANDMARK_Y - changeInY;
        double xAbsolute = LANDMARK_X + changeInX; 
        return new Point(xAbsolute,yAbsolute);
    }

    @Override
    public void updateState(RobotStateHistory pastRobotStates) {
        Point absolutePosition = getRobotPosition();
        DelayedPrinter.print("updating state", 5);
        test();
        pastRobotStates.statesAgo(0).set(SD.X, absolutePosition.x);
        pastRobotStates.statesAgo(0).set(SD.Y, absolutePosition.y);
    }

    @Override
    public Hashtable<SD, Double> getStdDevs() {
        return (Hashtable<SD, Double>) this.stdDevs.clone();
    }

    @Override
    public boolean canUpdate() { // for now, for simplicity, we will be assuming that the targets we find belong to the inner port
        limeLight.setCameraParams("pipeline", 0);
        boolean contourExists = limeLight.getTableData(limeLight.getCameraTable(), "tv") == 1;
        return contourExists;
    }
}