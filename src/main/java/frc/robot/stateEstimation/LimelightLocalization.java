package frc.robot.stateEstimation;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.shapes.Point;
import frc.robot.shapes.PolarPoint;
import frc.robot.helpers.Geo;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import java.util.Hashtable;

public class LimelightLocalization implements MaybeUpdater {
    public LimelightSubsystem limeLight;
    public double radialDistanceFromLoadingBay = 5; // To be tuned
    public double radialDistanceFromInnerPort  = 5; // To be tuned

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

    public Point getRobotPosition() {
        double distance = calculateDistance();
        double tx = limeLight.getTableData(limeLight.getCameraTable(), "tx");
        double xAbsolute = 0, yAbsolute = 0;
        if (tx > 0) {
            double xRelative = distance * Math.cos(Math.toRadians(tx));
            double yRelative = distance * Math.sin(Math.toRadians(tx));
            xAbsolute = LANDMARK_X - xRelative;
            yAbsolute = LANDMARK_Y - yRelative;
        } else {
            double xRelative = distance * Math.cos(Math.toRadians(tx));
            double yRelative = distance * Math.sin(Math.toRadians(tx));
            xAbsolute = xRelative - LANDMARK_X;
            yAbsolute = yRelative - LANDMARK_Y;
        }
        //PolarPoint relativePosition = new PolarPoint(distance, xAngle);
        //Point relativePoint = Geo.convertPolarToCartesian(relativePosition);
        return new Point(xAbsolute,yAbsolute);
    }

    @Override
    public void updateState(RobotStateHistory pastRobotStates) {
        Point absolutePosition = getRobotPosition();
        RobotState updatedState = new RobotState();
        updatedState.set(SD.X, absolutePosition.x);
        updatedState.set(SD.Y, absolutePosition.y);
        pastRobotStates.setCurrentState(updatedState);
    }

    @Override
    public Hashtable<SD, Double> getStdDevs() {
        return this.stdDevs;
    }

    public void printDistance() {
        double distance = calculateDistance();
        System.out.println("Target found at distance: " + distance);
        System.out.println("Ty is: " + Math.toRadians(limeLight.getTableData(limeLight.getCameraTable(), "ty")));
        limeLight.setCameraParams("distance", distance);
    }

    public void printPosition() {
        Point position = getRobotPosition();
        limeLight.setCameraParams("x", position.x);
        limeLight.setCameraParams("y", position.y);
    }

    @Override
    public boolean canUpdate() { // for now, for simplicity, we will be assuming that the targets we find belong to the inner port
        if (limeLight.getTableData(limeLight.getCameraTable(), "getpipe") == 1) {
            if (limeLight.getTableData(limeLight.getCameraTable(), "tv") == 1) {
                double distance = calculateDistance();
                return (distance <= radialDistanceFromInnerPort);           
            }
        }
        return false;
    }
}