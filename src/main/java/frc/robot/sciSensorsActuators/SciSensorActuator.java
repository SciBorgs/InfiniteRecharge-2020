package frc.robot.sciSensorsActuators;

import java.util.HashMap;

import frc.robot.robotState.RobotState.SD;

public interface SciSensorActuator<T> {
    public HashMap<T, SD> getSDMap();
    public void assignSD(T sciSD, SD sd);
    public SciUtils<T> getSciUtils();
    public String getDeviceName();
}