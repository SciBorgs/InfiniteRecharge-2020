package frc.robot.stateEstimation.explicit;

import frc.robot.Robot;
import frc.robot.helpers.DelayedPrinter;

import java.util.ArrayList;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.PortMap;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.logging.LogUpdater;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.stateEstimation.interfaces.*;

public class EncoderLocalization implements Updater, Model, LogUpdater {
    private static final double X_STD_DEV     = 0; // These are meant to be estimates
    private static final double Y_STD_DEV     = 0;
    private static final double ANGLE_STD_DEV = 0;
    private Hashtable<SD, Double> stdDevs;

    public EncoderLocalization(){
        this.stdDevs = new Hashtable<>();
        this.stdDevs.put(SD.X,     X_STD_DEV);
        this.stdDevs.put(SD.Y,     Y_STD_DEV);
        this.stdDevs.put(SD.Angle, ANGLE_STD_DEV);
        this.stdDevs.put(SD.GearShiftSolenoid, 0.0);
        this.stdDevs.put(SD.LeftWheelAngle, 0.0);
        this.stdDevs.put(SD.RightWheelAngle, 0.0);
        automateLogging();
    }

    @Override
    public Hashtable<SD, Double> getStdDevs(){return (Hashtable<SD, Double>) this.stdDevs.clone();}
    
    // Just multiplies the difference in angle by the wheel radius
    public double wheelRotationChange(SD wheelAngleSD, RobotStateHistory stateHistory){
        return StateInfo.getDifference(stateHistory, wheelAngleSD, 1) * DriveSubsystem.WHEEL_RADIUS;
    }

    public RobotState nextPosition(double x, double y, double theta, ArrayList<Double> wheelDistances, double deltaTheta){
        // Works for all forms of drive where the displacement is the average of the movement vectors over the wheels
        double newTheta = theta + deltaTheta;
        RobotState robotState = new RobotState();
        
        double avgTheta = (theta + newTheta)/2; // we use the avgtheta, as it reduces error tremendously
        for(double wheelDistance : wheelDistances){
            x += wheelDistance * Math.cos(avgTheta) / wheelDistances.size();
            y += wheelDistance * Math.sin(avgTheta) / wheelDistances.size();
        }
        robotState.set(SD.X, x);
        robotState.set(SD.Y, y);
        robotState.set(SD.Angle, newTheta);
        return robotState;
    }

    @Override
    public void updateState(RobotStateHistory stateHistory){
        RobotState state = stateHistory.statesAgo(1);
        ArrayList<Double> wheelChanges = new ArrayList<>();
        wheelChanges.add(wheelRotationChange(SD.LeftWheelAngle,  stateHistory));
        wheelChanges.add(wheelRotationChange(SD.RightWheelAngle, stateHistory));
        double thetaChange = StateInfo.getDifference(stateHistory, SD.PigeonAngle, 1);
        RobotState newPosition = 
            nextPosition(state.get(SD.X), state.get(SD.Y), state.get(SD.Angle), wheelChanges, thetaChange);
        stateHistory.currentState().incorporateOtherState(newPosition); 
    }

    @Override
    public void updateRobotState(){
        updateState(Robot.stateHistory);
    }

    @Override
    public Iterable<SD> getSDs(){return this.stdDevs.keySet();}
    
	public void periodicLog(){
	}

    public void printWheelAngles(){
        DelayedPrinter.print("leftWheelAngle: " + Robot.get(SD.LeftWheelAngle));
        DelayedPrinter.print("rightWheelAngle: " + Robot.get(SD.RightWheelAngle));
    }
}