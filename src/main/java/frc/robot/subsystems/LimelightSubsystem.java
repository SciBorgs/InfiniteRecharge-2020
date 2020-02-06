package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem implements Subsystem{

    public final static double IMAGE_WIDTH = Math.toRadians(27.); // In degrees
    public final static double IMAGE_HEIGHT = Math.toRadians(20.5); // In degrees
	private final String FILENAME = "LimelightSubsystem.java";

    public NetworkTable getCameraTable(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    public double getTableData(NetworkTable table, String variable){ 
        // According to API, should get a given variable (x1,a2... etc.)
        return table.getEntry(variable).getDouble(0);
    }
    public void setCameraParams(String param, int setting){ // According to API, should set a given param (camMode, pipeline... etc.)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(param).setNumber(setting);
    }
    public void setCameraParams(String param, double setting){ // According to API, should set a given param (camMode, pipeline... etc.)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(param).setNumber(setting);
    }
    public boolean contourExists(){
        return getTableData(getCameraTable(), "tv") == 1;
    }

    public void periodicLog(){
    }
}