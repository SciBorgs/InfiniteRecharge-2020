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
    private static final double CAMERA_MOUNTING_ANGLE      = Math.toRadians(35.6);
    private static final double OUTER_PORT_HEIGHT          = Utils.inchesToMeters(90);

    private static final double LANDMARK_X = 0;
    private static final double LANDMARK_Y = 0;

    private Hashtable<SD, Double> stdDevs;

    public LimelightLocalization() {
        this.limeLight = Robot.limelightSubsystem;
        this.stdDevs = new Hashtable<>();
        this.stdDevs.put(SD.X, 0.0);
        this.stdDevs.put(SD.Y, 0.0);
        this.stdDevs.put(SD.DistanceToPort, 0.0);
    }

    public double calculateDistance() {
        double yAngle = Math.toRadians(limeLight.getTableData(limeLight.getCameraTable(), "ty"));
        return (OUTER_PORT_HEIGHT - CAMERA_ABOVE_GROUND_HEIGHT)/Math.tan(yAngle + CAMERA_MOUNTING_ANGLE); 
    }

    public Point getRobotPosition() {
        double distance = calculateDistance();
        double tx = limeLight.getTableData(limeLight.getCameraTable(), "tx");
        double totalAngle = Math.toRadians(tx) + Robot.get(SD.Angle);
        double changeInY = distance * Math.sin(totalAngle);
        double changeInX = Math.sqrt(distance * distance - changeInY * changeInY);
        double yAbsolute = LANDMARK_Y - changeInY;
        double xAbsolute = LANDMARK_X + changeInX; 
        return new Point(xAbsolute,yAbsolute);
    }

    @Override
    public void updateState(RobotStateHistory stateHistory) {
        Point absolutePosition = getRobotPosition();
        stateHistory.statesAgo(0).set(SD.DistanceToPort, calculateDistance());
        stateHistory.statesAgo(0).set(SD.X, absolutePosition.x);
        stateHistory.statesAgo(0).set(SD.Y, absolutePosition.y);
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