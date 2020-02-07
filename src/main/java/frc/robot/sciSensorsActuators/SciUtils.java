package frc.robot.sciSensorsActuators;

import frc.robot.Robot;
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
    public RobotStateHistory getDeviceData(){
        RobotStateHistory newStateHistory = Robot.stateHistory.copy();
        for(RobotState robotState : newStateHistory){
            robotState.cutDownTo(getSDs());
        }
        return newStateHistory;
    }
}