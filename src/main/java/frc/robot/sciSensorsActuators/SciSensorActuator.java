package frc.robot.sciSensorsActuators;

import java.util.HashMap;

import frc.robot.Robot;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;

public interface SciSensorActuator<T> {
    public HashMap<T, SD> getSDMap();
    public String getDeviceName();

    default public void sciSet(T sciSD, double v){
        if (isTracked(sciSD)){
            Robot.set(getSDMap().get(sciSD), v);
        }
    }

    default public void assignSD(T sciSD, SD sd){
        getSDMap().put(sciSD, sd);
        Robot.set(sd, 0);
    }

    default public boolean isTracked(T sciSD){
        return getSDMap().containsKey(sciSD);
    }

    default public double sciGet(T sciSD){
        return Robot.get(getSDMap().get(sciSD));
    }

    default public SD sdOf(T sciSD) {
        return getSDMap().get(sciSD);
    }

    default public Iterable<SD> getSDs(){
        return getSDMap().values();
    }

    default public void logAllSDs(){
        for(SD sd : getSDs()){
            Robot.addSDToLog(sd);
        }
    }
    default public RobotState getDeviceData(RobotState robotState){
        return robotState.cutDownIntoNew(getSDs());
    }
    default public RobotStateHistory getAllDeviceData(){
        RobotStateHistory newStateHistory = Robot.stateHistory.copy();
        for(RobotState robotState : newStateHistory){
            robotState.cutDownTo(getSDs());
        }
        return newStateHistory;
    }

    default public void printAllData(){
        System.out.println("Current " + getDeviceName() + " data: ");
        for(Pair<SD, Double> data : getDeviceData(Robot.getState())){
            System.out.println(data.first + " = " + data.second);
        }
    }
}