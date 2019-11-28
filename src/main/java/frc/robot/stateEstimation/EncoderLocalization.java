package frc.robot.stateEstimation;

import frc.robot.Robot;

import java.util.ArrayList;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.robotState.*;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.logging.Logger.DefaultValue;

public class EncoderLocalization implements Updater, Model {

    public static final double ORIGINAL_ANGLE = Math.PI/2, ORIGINAL_X = 0, ORIGINAL_Y = 0;
    public static final double INTERVAL_LENGTH = .02; // Seconds between each tick for commands
    private final String FILENAME = "RobotPosition.java";
    private static final double X_STD_DEV     = 0; // These are meant to be estimates
    private static final double Y_STD_DEV     = 0;
    private static final double ANGLE_STD_DEV = 0;
    private Hashtable<SD, Double> stdDevs;

    private SciPigeon pigeon;
    private TalonSRX pigeonTalon;

    private class WheelChangeInfo{
        public double rotationChange, angle;
        public WheelChangeInfo(double rotationChange, double angle){
            this.rotationChange = rotationChange;
            this.angle = angle;
        }
    }

    public EncoderLocalization(){
        this.pigeonTalon = new TalonSRX(PortMap.PIGEON_TALON);
        this.pigeon      = new SciPigeon(pigeonTalon);

        this.stdDevs = new Hashtable<>();
        this.stdDevs.put(SD.X,     X_STD_DEV);
        this.stdDevs.put(SD.Y,     Y_STD_DEV);
        this.stdDevs.put(SD.Angle, ANGLE_STD_DEV);
    }

    public SciPigeon getPigeon() {return this.pigeon;}

    @Override
    public Hashtable<SD, Double> getStdDevs(){return this.stdDevs;}
    
    public double wheelRotationChange(SD wheelAngleSD, RobotStateHistory stateHistory){
        return StateInfo.getDifference(stateHistory, wheelAngleSD, 1) * DriveSubsystem.WHEEL_RADIUS;
    }

    public WheelChangeInfo newWheelChangeInfo(double rotationChange, double angle){
        return new WheelChangeInfo(rotationChange, angle);
    }

    public RobotState nextPosition(double x, double y, double theta, ArrayList<WheelChangeInfo> allChangeInfo){
        // Works for all forms of drive where the displacement is the average of the movement vectors over the wheels
        double newTheta = pigeon.getAngle();
        double avgTheta = (theta + newTheta)/2;
        int wheelAmount = allChangeInfo.size();
        for(WheelChangeInfo wheelChangeInfo : allChangeInfo){
            x += wheelChangeInfo.rotationChange * Math.cos(avgTheta + wheelChangeInfo.angle) / wheelAmount;
            y += wheelChangeInfo.rotationChange * Math.sin(avgTheta + wheelChangeInfo.angle) / wheelAmount;
        }
        RobotState state = new RobotState();
        state.set(SD.X, x);
        state.set(SD.Y, y);
        state.set(SD.Angle, newTheta);
        return state;
    }
 
    public RobotState nextPosTankPigeon(double x, double y, double theta, double leftChange, double rightChange) {
        // This assumes tank drive and you want to use the pigeon for calculating your angle
        // Takes a pos (x,y,theta), a left side Δx and a right side Δx and returns an x,y,theta array
        ArrayList<WheelChangeInfo> allChangeInfo = new ArrayList<>();
        allChangeInfo.add(new WheelChangeInfo(leftChange,  0));
        allChangeInfo.add(new WheelChangeInfo(rightChange, 0)); // the zeros represent that they aren't turned
        return nextPosition(x,y,theta,allChangeInfo);
    }

    @Override
    public RobotState updateState(RobotStateHistory pastStates){
        RobotState state = pastStates.currentState();
        RobotState newPosition = 
            nextPosTankPigeon(state.get(SD.X), state.get(SD.Y), state.get(SD.Angle), 
                wheelRotationChange(SD.LeftWheelAngle,  pastStates), 
                wheelRotationChange(SD.RightWheelAngle, pastStates));
        return pastStates.currentState().incorporateIntoNew(newPosition); 
    }

    @Override
    public RobotState updatedRobotState(){
        return updateState(Robot.stateHistory);
    }

    @Override
    public Iterable<SD> getSDs(){return this.stdDevs.keySet();}
    
	public void periodicLog(){
        Robot.logger.addData(FILENAME, "robot X",     Robot.get(SD.X),     DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "robot y",     Robot.get(SD.Y),     DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "robot angle", Robot.get(SD.Angle), DefaultValue.Previous);
	}

    public void printPosition(){
        System.out.println("X: " + Robot.get(SD.X));
        System.out.println("Y: " + Robot.get(SD.Y));
        System.out.println("Angle: " + Math.toDegrees(Robot.get(SD.Angle)));
    }
}