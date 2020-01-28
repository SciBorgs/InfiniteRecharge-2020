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
    private static final double LOADING_BAY_HEIGHT         = Utils.inchesToMeters(11);

    private static final double PIXEL_TO_NORMALIZED_X_RATIO  = 1/160;
    private static final double PIXEL_TO_NORMALIZED_Y_RATIO  = 1/120;

    private static final double PIXEL_TO_NORMALIZED_X_OFFSET  = 159.5;
    private static final double PIXEL_TO_NORMALIZED_Y_OFFSET  = 119.5;

    private static final double CAMERA_HORIZONTAL_POV = Math.toRadians(54);
    private static final double CAMERA_VERTICAL_POV   = Math.toRadians(41);

    private static final double LANDMARK_X = -10;
    private static final double LANDMARK_Y =  2;

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

    public Point getRobotPosition(PolarPoint polarP, Point landmarkLocation) {
        Point relativePoint = Geo.convertPolarToCartesian(polarP);
        double xAbsolute = relativePoint.x + landmarkLocation.x;
        double yAbsolute = relativePoint.y + landmarkLocation.y;
        return new Point(xAbsolute,yAbsolute);
    }

    @Override
    public void updateState(RobotStateHistory pastRobotStates) {
        double distance = calculateDistance();
        double xAngle = limeLight.getTableData(limeLight.getCameraTable(), "tx");
        PolarPoint relativePosition = new PolarPoint(distance, xAngle);
        Point absolutePosition = getRobotPosition(relativePosition, new Point(LANDMARK_X, LANDMARK_Y));
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
    }

    @Override
    public boolean canUpdate() { // for now, for simplicity, we will be assuming that the targets we find belong to the inner port
        if (limeLight.getTableData(limeLight.getCameraTable(), "getpipe") == 1) {
            if (limeLight.getTableData(limeLight.getCameraTable(), "tv") == 1) {
                double distance = calculateDistance();
                System.out.println("Target found at distance: " + distance);
                return (distance <= radialDistanceFromInnerPort);           
            }
        }
        return false;
    }
}