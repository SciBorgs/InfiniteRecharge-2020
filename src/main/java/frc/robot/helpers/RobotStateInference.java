package frc.robot.helpers;

import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.RS;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

public class RobotStateInference{

    public static final int ANGULAR_VELOCITY_PRECISION = 5;
    public static final int X_VELOCITY_PRECISION = 5;
    public static final int Y_VELOCITY_PRECISION = 5;
    public static final int WHEEL_SPEED_PRECISION = 5;

    public static double getDifference(ArrayList<RobotState> states, RS rs, int ticksBack1, int ticksBack2){
        return states.get(ticksBack1).get(rs) - states.get(ticksBack2).get(rs);
    }
    public static double getDifference(ArrayList<RobotState> states, RS rs, int ticksBack){
        return getDifference(states, rs, 0, ticksBack);
    }
    public static double getRateOfChange(ArrayList<RobotState> states, RS rs, int ticksBack1, int ticksBack2) {
        return (states.get(ticksBack1).get(rs) - states.get(ticksBack2).get(rs)) / (ticksBack2 - ticksBack1);
    }
    public static double getRateOfChange(ArrayList<RobotState> states, RS rs, int ticksBack) {
        return getRateOfChange(states, rs, 0, ticksBack);
    }

    public static double getAngularVelocity(ArrayList<RobotState> states){
        return getRateOfChange(states, RS.Angle, ANGULAR_VELOCITY_PRECISION);
    }
    public static double getXVelocity(ArrayList<RobotState> states){
        return getRateOfChange(states, RS.X, X_VELOCITY_PRECISION);
    }
    public static double getYVelocity(ArrayList<RobotState> states){
        return getRateOfChange(states, RS.Y, Y_VELOCITY_PRECISION);
    }
    public static double getSpeedSquared(ArrayList<RobotState> states){
        return Math.pow(getXVelocity(states), 2) + Math.pow(getYVelocity(states), 2);
    }
    public static double getSpeed(ArrayList<RobotState> states){
        return Math.sqrt(getSpeedSquared(states));
    }
    public static double getWheelSpeed(ArrayList<RobotState> states, CANSparkMax wheel){
        return getRateOfChange(states, Robot.driveSubsystem.sparkToWheelRS.get(wheel), WHEEL_SPEED_PRECISION);
    }

    public static double getAngularVelocity() {return getAngularVelocity(Robot.robotStates);}
    public static double getXVelocity()       {return getXVelocity(      Robot.robotStates);}
    public static double getYVelocity()       {return getYVelocity(      Robot.robotStates);}
    public static double getSpeedSquared()    {return getSpeedSquared(   Robot.robotStates);}
    public static double getSpeed()           {return getSpeed(          Robot.robotStates);}

    public static double getWheelSpeed(CANSparkMax wheel){return getWheelSpeed(Robot.robotStates, wheel);}

}