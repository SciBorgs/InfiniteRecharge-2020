package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;

import com.revrobotics.CANSparkMax;

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
            (ticksBack2 - ticksBack1);
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
    public static double getWheelSpeed(RobotStateHistory stateHistory, CANSparkMax wheel){
        return getRateOfChange(stateHistory, Robot.driveSubsystem.sparkToWheelAngleSD.get(wheel), WHEEL_SPEED_PRECISION);
    }
    public static double getAvgWheelInput(RobotState state){
        return (state.get(SD.LeftSparkVal) + state.get(SD.RightSparkVal)) / 2;
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

    public static double getAngularVelocity() {return getAngularVelocity(Robot.stateHistory);}
    public static double getXVelocity()       {return getXVelocity(      Robot.stateHistory);}
    public static double getYVelocity()       {return getYVelocity(      Robot.stateHistory);}
    public static double getSpeedSquared()    {return getSpeedSquared(   Robot.stateHistory);}
    public static double getSpeed()           {return getSpeed(          Robot.stateHistory);}

    public static double getAvgWheelInput() {return getAvgWheelInput(Robot.getState());}

    public static double getWheelSpeed(CANSparkMax wheel){return getWheelSpeed(Robot.stateHistory, wheel);}

}