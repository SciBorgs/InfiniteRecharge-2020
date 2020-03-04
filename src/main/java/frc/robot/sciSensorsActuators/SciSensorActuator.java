package frc.robot.sciSensorsActuators;

import java.util.HashMap;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;

public interface SciSensorActuator<SciSD> {
    public HashMap<SciSD, SD> getSDMap();
    public String getDeviceName();

    public default boolean sciIgnore(int deviceID){
        return PortMap.ignoreID(deviceID);
    }

    public default void sciSet(SciSD sciSD, double v){
        if (isTracked(sciSD)){
            Robot.set(getSDMap().get(sciSD), v);
        }
    }

    public default void assignSD(SciSD sciSD, SD sd){
        getSDMap().put(sciSD, sd);
        Robot.set(sd, 0);
    }

    public default boolean isTracked(SciSD sciSD){
        return getSDMap().containsKey(sciSD);
    }

    public default double sciGet(SciSD sciSD){
        return Robot.get(getSDMap().get(sciSD));
    }

    public default SD sdOf(SciSD sciSD) {
        return getSDMap().get(sciSD);
    }

    public default Iterable<SD> getSDs(){
        return getSDMap().values();
    }

    public default void logAllSDs(){
        for(SD sd : getSDs()){
            Robot.addSDToLog(sd);
        }
    }
    public default RobotState getDeviceData(RobotState robotState){
        return robotState.cutDownIntoNew(getSDs());
    }
    public default RobotStateHistory getAllDeviceData(){
        RobotStateHistory newStateHistory = Robot.stateHistory.copy();
        for(RobotState robotState : newStateHistory){
            robotState.cutDownTo(getSDs());
        }
        return newStateHistory;
    }

    public default void printAllData(){
        System.out.println("Current " + getDeviceName() + " data: ");
        for(Pair<SD, Double> data : getDeviceData(Robot.getState())){
            System.out.println(data.first + " = " + data.second);
        }
    }
}