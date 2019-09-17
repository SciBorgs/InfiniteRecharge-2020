package frc.robot.helpers;

import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotStates;
import frc.robot.RobotState.RS;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

public class StateInf{

    public static final int ANGULAR_VELOCITY_PRECISION = 5;
    public static final int X_VELOCITY_PRECISION = 5;
    public static final int Y_VELOCITY_PRECISION = 5;
    public static final int WHEEL_SPEED_PRECISION = 5;

    public static double getDifference(RobotStates states, RS rs, int ticksBack1, int ticksBack2){
        return states.statesAgo(ticksBack1).get(rs) - states.statesAgo(ticksBack2).get(rs);
    }
    public static double getDifference(RobotStates states, RS rs, int ticksBack){
        return getDifference(states, rs, 0, ticksBack);
    }
    public static double getRateOfChange(RobotStates states, RS rs, int ticksBack1, int ticksBack2) {
        return (states.statesAgo(ticksBack1).get(rs) - states.statesAgo(ticksBack2).get(rs)) / (ticksBack2 - ticksBack1);
    }
    public static double getRateOfChange(RobotStates states, RS rs, int ticksBack) {
        return getRateOfChange(states, rs, 0, ticksBack);
    }

    public static double getAngularVelocity(RobotStates states){
        return getRateOfChange(states, RS.Angle, ANGULAR_VELOCITY_PRECISION);
    }
    public static double getXVelocity(RobotStates states){
        return getRateOfChange(states, RS.X, X_VELOCITY_PRECISION);
    }
    public static double getYVelocity(RobotStates states){
        return getRateOfChange(states, RS.Y, Y_VELOCITY_PRECISION);
    }
    public static double getSpeedSquared(RobotStates states){
        return Math.pow(getXVelocity(states), 2) + Math.pow(getYVelocity(states), 2);
    }
    public static double getSpeed(RobotStates states){
        return Math.sqrt(getSpeedSquared(states));
    }
    public static double getWheelSpeed(RobotStates states, CANSparkMax wheel){
        return getRateOfChange(states, Robot.driveSubsystem.sparkToWheelAngleRS.get(wheel), WHEEL_SPEED_PRECISION);
    }
    public static double getAvgWheelInput(RobotState state){
        return (state.get(RS.LeftSparkVal) + state.get(RS.RightSparkVal)) / 2;
    }

    public static double getAngularVelocity() {return getAngularVelocity(Robot.robotStates);}
    public static double getXVelocity()       {return getXVelocity(      Robot.robotStates);}
    public static double getYVelocity()       {return getYVelocity(      Robot.robotStates);}
    public static double getSpeedSquared()    {return getSpeedSquared(   Robot.robotStates);}
    public static double getSpeed()           {return getSpeed(          Robot.robotStates);}

    public static double getAvgWheelInput() {return getAvgWheelInput(Robot.getState());}

    public static double getWheelSpeed(CANSparkMax wheel){return getWheelSpeed(Robot.robotStates, wheel);}

}