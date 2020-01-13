package frc.robot.stateEstimation;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.shapes.Point;
import frc.robot.shapes.PolarPoint;
import frc.robot.helpers.Geo;

public class LimelightLocalization {
    public LimelightSubsystem limeLight;
    public double radialDistanceFromLoadingBay = 5; // To be tuned
    public double radialDistanceFromInnerPort  = 5; // To be tuned

    private static final double CAMERA_ABOVE_GROUND_HEIGHT = Utils.inchesToMeters(5);
    private static final double CAMERA_MOUNTING_ANGLE      = Utils.inchesToMeters(5);
    private static final double OUTER_PORT_HEIGHT          = Utils.inchesToMeters(98.25);
    private static final double LOADING_BAY_HEIGHT         = Utils.inchesToMeters(11);

    private static final double CAMERA_HORIZONTAL_POV = Math.toRadians(54);
    private static final double CAMERA_VERTICAL_POV   = Math.toRadians(41);

    public LimelightLocalization(LimelightSubsystem limeLight) {
        this.limeLight = limeLight;
    }

    public double calculateDistance(double heightOne, double heightTwo, double angleOne, double angleTwo) {
        return (heightTwo - heightOne)/Math.tan(angleOne + angleTwo);
    }

    public Point getRobotPosition(PolarPoint polarP, Point landmarkLocation) {
        Point relativePoint = Geo.convertPolarToCartesian(polarP);
        double xAbsolute = relativePoint.x + landmarkLocation.x;
        double yAbsolute = relativePoint.y + landmarkLocation.y;
        return new Point(xAbsolute,yAbsolute);
    }

    public double getYAngleToPixel(double pixelY) {
        double normalizedY = (1/120) * (119.5 - pixelY);
        double viewPlaneHeight = 2 * Math.tan(CAMERA_VERTICAL_POV/2);
        double y = viewPlaneHeight/2 * normalizedY;
        double yAngle =  Math.atan2(1,y);
        return yAngle;
    }

    public double getXAngleToPixel(double pixelX) {
        double normalizedX =  (1/160) * (pixelX - 159.5);
        double viewPlaneWidth = 2 * Math.tan(CAMERA_HORIZONTAL_POV/2);
        double x = viewPlaneWidth/2 * normalizedX;
        double xAngle = Math.atan2(1,x);
        return xAngle;
    }

    public boolean isInRange() { // for now, for simplicity, we will be assuming that the targets we find belong to the inner port
        if (limeLight.getTableData(limeLight.getCameraTable(), "getpipe") == 1) {
            if (limeLight.getTableData(limeLight.getCameraTable(), "tv") == 1) {
                double yAngle = getYAngleToPixel(limeLight.getTableData(limeLight.getCameraTable(), "ty"));
                double distance = calculateDistance(CAMERA_ABOVE_GROUND_HEIGHT, OUTER_PORT_HEIGHT, CAMERA_MOUNTING_ANGLE, yAngle);
                System.out.println("Target found: " + distance);
                return (distance <= radialDistanceFromInnerPort);           
            }
        }
        return false;
    }
}