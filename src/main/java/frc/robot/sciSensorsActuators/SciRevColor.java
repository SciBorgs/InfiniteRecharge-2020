package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.I2C.Port;

import frc.robot.colors.*;
import frc.robot.helpers.Colors;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciRevColor.SciRevColorSD;

import java.util.HashMap;

import com.revrobotics.ColorSensorV3;

public class SciRevColor extends ColorSensorV3 implements RobotStateUpdater, SciSensorActuator<SciRevColorSD> {
    public static enum SciRevColorSD {H, S, V}
    public HashMap<SciRevColorSD, SD> sdMap;
    private Port port;
    public SciRevColor(Port port) {
        super(port);
        this.port = port;
        automateStateUpdating();
    }

    @Override
    public HashMap<SciRevColorSD, SD> getSDMap() { return this.sdMap; }
    @Override
    public String getDeviceName() { return "REV Robotics Color Sensor V3  " + port.name(); }

    public CMYK getCMYK() {
        return Colors.toCMYK(getColor());
    }

    public HSV getHSV(){
        return Colors.toHSV(getColor());
    }

    public RGB getRGB() {
        return Colors.toRGB(getColor());
    }
    
    @Override
    public void updateRobotState() {
        HSV color = getHSV();
        sciSet(SciRevColorSD.H, color.getH());
        sciSet(SciRevColorSD.S, color.getS());
        sciSet(SciRevColorSD.V, color.getV());
    }
}