package frc.robot.stateEstimation;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.shapes.Point;
public class LimelightLocalization {
    public LimelightSubsystem limeLight;
    public double radialDistanceFromLoadingBay = 5;
    public double radialDistanceFromInnerPort = 5;

    private static final double CAMERA_ABOVE_GROUND_HEIGHT = 5;
    private static final double CAMERA_HORIZONTAL_POV = 54 * Math.PI/180;;
    private static final double CAMERA_VERTICAL_POV = 41 * Math.PI/180;
    private static final double CAMERA_MOUNTING_ANGLE = 5;
    private static final double INNER_PORT_HEIGHT = Utils.inchesToMeters(98.25);
    private static final double LOADING_BAY_HEIGHT = Utils.inchesToMeters(11);

    public LimelightLocalization(LimelightSubsystem limeLight) {
        this.limeLight = limeLight;
    }

    public double calculateDistance(double heightOne, double heightTwo, double angleOne, double angleTwo) {
        return (heightTwo - heightOne)/Math.tan(angleOne + angleTwo);
    }

    public Point convertToCartesian(double r, double theta) {
        double x = r * Math.cos(theta);
        double y = r * Math.sin(theta);
        return new Point(x,y);
    }

    public Point getRobotPosition(double r, double theta, Point landmarkLocation) {
        Point relativePoint = convertToCartesian(r, theta);
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

    public boolean isInRange() { // for now, for simplicity, we will be assuming that the targets we find belong to the inner port
        if (limeLight.getTableData(limeLight.getCameraTable(), "getpipe") == 1) {
            if (limeLight.getTableData(limeLight.getCameraTable(), "tv") == 1) {
                double yAngle = limeLight.getTableData(limeLight.getCameraTable(), "ty");
                double distance = calculateDistance(CAMERA_ABOVE_GROUND_HEIGHT, INNER_PORT_HEIGHT, CAMERA_MOUNTING_ANGLE, yAngle);
                System.out.println("Target found");
                return (distance <= radialDistanceFromInnerPort);           
            }
        }
        return false;
    }
}