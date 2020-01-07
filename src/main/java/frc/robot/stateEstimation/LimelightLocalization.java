package frc.robot.stateEstimation;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightLocalization {
    public LimelightSubsystem limeLight;
    public double radialDistanceFromLoadingBay = 5;
    public double radialDistanceFromInnerPort = 5;

    public static final double CAMERA_ABOVE_GROUND_HEIGHT = 5;
    public static final double CAMERA_MOUNTING_ANGLE = 5;
    public static final double INNER_PORT_HEIGHT = Utils.inchesToMeters(98.25);
    public static final double LOADING_BAY_HEIGHT = Utils.inchesToMeters(11);

    public LimelightLocalization(LimelightSubsystem limeLight) {
        this.limeLight = limeLight;
    }

    public static double calculateDistance(double heightOne, double heightTwo, double angleOne, double angleTwo) {
        return (heightTwo - heightOne)/Math.tan(angleOne + angleTwo);
    }

    public boolean isInRange() {
        double angleToTarget = limeLight.getTableData(limeLight.getCameraTable(), variable);
        if (limelight.getTableData(limeLight.getCameraTable(), "getpipe") == 1) {
            if (calculateDistance(CAMERA_ABOVE_GROUND_HEIGHT, heightTwo, CAMERA_MOUNTING_ANGLE, angleTwo))
        }
    }
}