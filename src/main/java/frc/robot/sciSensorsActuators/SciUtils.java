package frc.robot.sciSensorsActuators;

import frc.robot.Robot;
import frc.robot.dataTypes.Pair;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotStateHistory;
import frc.robot.robotState.RobotState.SD;

public class SciUtils<SciSD> {
    
    private SciSensorActuator<SciSD> sciSensorActuator;

    public SciUtils(SciSensorActuator<SciSD> sciSensorActuator){
        this.sciSensorActuator = sciSensorActuator;
    }

    public void sciSet(SciSD sciSD, double v){
        if (isTracked(sciSD)){
            Robot.set(this.sciSensorActuator.getSDMap().get(sciSD), v);
        }
    }

    public void assignSD(SciSD sciSD, SD sd){
        this.sciSensorActuator.getSDMap().put(sciSD, sd);
        Robot.set(sd, 0);
    }

    public boolean isTracked(SciSD sciSD){
        return this.sciSensorActuator.getSDMap().containsKey(sciSD);
    }

    public double sciGet(SciSD sciSD){
        return Robot.get(this.sciSensorActuator.getSDMap().get(sciSD));
    }

    public SD sdOf(SciSD sciSD) {
        return this.sciSensorActuator.getSDMap().get(sciSD);
    }

    public Iterable<SD> getSDs(){
        return this.sciSensorActuator.getSDMap().values();
    }

    public void logAllSDs(){
        for(SD sd : getSDs()){
            Robot.addSDToLog(sd);
        }
    }
    public RobotState getDeviceData(RobotState robotState){
        return robotState.cutDownIntoNew(getSDs());
    }
    public RobotStateHistory getAllDeviceData(){
        RobotStateHistory newStateHistory = Robot.stateHistory.copy();
        for(RobotState robotState : newStateHistory){
            robotState.cutDownTo(getSDs());
        }
        return newStateHistory;
    }

    public void printAllData(){
        System.out.println("Current " + this.sciSensorActuator.getDeviceName() + " data: ");
        for(Pair<SD, Double> data : getDeviceData(Robot.getState())){
            System.out.println(data.first + " = " + data.second);
        }
    }
}