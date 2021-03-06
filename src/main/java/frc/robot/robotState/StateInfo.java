package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;

public class StateInfo{

    public static final int ANGULAR_VELOCITY_PRECISION = 5;
    public static final int X_VELOCITY_PRECISION = 5;
    public static final int Y_VELOCITY_PRECISION = 5;
    public static final int WHEEL_SPEED_PRECISION = 5;

    public static double getDifference(RobotStateHistory stateHistory, SD sd, int ticksBack1, int ticksBack2){
        return stateHistory.statesAgo(ticksBack1).get(sd) - stateHistory.statesAgo(ticksBack2).get(sd);
    }
    public static double getDifference(RobotStateHistory stateHistory, SD sd, int ticksBack){
        return getDifference(stateHistory, sd, 0, ticksBack);
    }
    public static double getFullDifference(RobotStateHistory stateHistory, SD sd){
        return getDifference(stateHistory, sd, stateHistory.numberOfStates() - 1);
    }

    public static double getRateOfChange(RobotStateHistory stateHistory, SD sd, int ticksBack1, int ticksBack2) {
        return (stateHistory.statesAgo(ticksBack1).get(sd) - stateHistory.statesAgo(ticksBack2).get(sd)) / 
            (stateHistory.statesAgo(ticksBack1).get(SD.Time) - stateHistory.statesAgo(ticksBack2).get(SD.Time));
    }
    public static double getRateOfChange(RobotStateHistory stateHistory, SD sd, int ticksBack) {
        return getRateOfChange(stateHistory, sd, 0, ticksBack);
    }

    public static double getAngularVelocity(RobotStateHistory stateHistory){
        return getRateOfChange(stateHistory, SD.PigeonAngle, ANGULAR_VELOCITY_PRECISION);
    }
    public static double getXVelocity(RobotStateHistory stateHistory){
        return getRateOfChange(stateHistory, SD.X, X_VELOCITY_PRECISION);
    }
    public static double getYVelocity(RobotStateHistory stateHistory){
        return getRateOfChange(stateHistory, SD.Y, Y_VELOCITY_PRECISION);
    }
    public static double getSpeedSquared(RobotStateHistory stateHistory){
        return Math.pow(getXVelocity(stateHistory), 2) + Math.pow(getYVelocity(stateHistory), 2);
    }
    public static double getSpeed(RobotStateHistory stateHistory){
        return Math.sqrt(getSpeedSquared(stateHistory));
    }
    public static double getAvgWheelInput(RobotState state){
        return (state.get(SD.LeftWheelVal) + state.get(SD.RightWheelVal)) / 2;
    }

    public static double getDifference(SD sd, int ticksBack1, int ticksBack2) {
        return getDifference(Robot.stateHistory, sd, ticksBack1, ticksBack2);
    }
    public static double getDifference(SD sd, int ticksBack) {
        return getDifference(Robot.stateHistory, sd, ticksBack);
    }
    public static double getFullDifference(SD sd){
        return getFullDifference(Robot.stateHistory, sd);
    }
    public static double getRateOfChange(SD sd, int ticksBack){
        return getRateOfChange(Robot.stateHistory, sd, ticksBack);
    }

    public static double avgValue(RobotStateHistory stateHistory, SD sd, int numValues) {
        double sum = 0;
        for(int i = 0; i < numValues; i++){
            sum += stateHistory.statesAgo(i).get(sd);
        }
        return sum / numValues;
    }

    public static double avgValue(SD sd, int numValues){
        return avgValue(Robot.stateHistory, sd, numValues);
    }

    public static double getAngularVelocity() {return getAngularVelocity(Robot.stateHistory);}
    public static double getXVelocity()       {return getXVelocity(      Robot.stateHistory);}
    public static double getYVelocity()       {return getYVelocity(      Robot.stateHistory);}
    public static double getSpeedSquared()    {return getSpeedSquared(   Robot.stateHistory);}
    public static double getSpeed()           {return getSpeed(          Robot.stateHistory);}

    public static double getAvgWheelInput() {return getAvgWheelInput(Robot.getState());}
}