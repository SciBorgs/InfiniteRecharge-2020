package frc.robot.stateEstimation;

import frc.robot.Robot;

import java.util.ArrayList;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.RobotState;
import frc.robot.RobotStateHistory;
import frc.robot.RobotState.SD;
import frc.robot.helpers.Pigeon;
import frc.robot.logging.Logger.DefaultValue;

public class EncoderLocalization implements Updater, Model {

    public static final double WHEEL_RADIUS = Utils.inchesToMeters(3);
    public static final double ROBOT_RADIUS = Utils.inchesToMeters(15.945); // Half the distance from wheel to wheel

    public static final double ORIGINAL_ANGLE = Math.PI/2, ORIGINAL_X = 0, ORIGINAL_Y = 0;
    public static final double INTERVAL_LENGTH = .02; // Seconds between each tick for commands
    private final String FILENAME = "RobotPosition.java";
    private static final double X_STD_DEV     = 0; // These are meant to be estimates
    private static final double Y_STD_DEV     = 0;
    private static final double ANGLE_STD_DEV = 0;
    private Hashtable<SD, Double> stdDevs;

    public Pigeon pigeon;
    private TalonSRX pigeonTalon;

    public EncoderLocalization(){
        this.pigeonTalon = new TalonSRX(PortMap.PIGEON_TALON);
        this.pigeon      = new Pigeon(pigeonTalon);

        this.stdDevs = new Hashtable<>();
        this.stdDevs.put(SD.X,     X_STD_DEV);
        this.stdDevs.put(SD.Y,     Y_STD_DEV);
        this.stdDevs.put(SD.Angle, ANGLE_STD_DEV);
        this.stdDevs.put(SD.GearShiftSolenoid, 0.0);
        this.stdDevs.put(SD.LeftWheelAngle, 0.0);
        this.stdDevs.put(SD.RightWheelAngle, 0.0);
    }

    public TalonSRX[] getTalons() {
        return new TalonSRX[]{this.pigeonTalon};
    }

    public Pigeon                getPigeon() {return this.pigeon;}
    @Override
    public Hashtable<SD, Double> getStdDevs(){return this.stdDevs;}
    
    public double wheelRotationChange(SD wheelAngleSD, RobotStateHistory stateHistory){
        return getWheelPosition(wheelAngleSD, stateHistory.statesAgo(0)) - 
               getWheelPosition(wheelAngleSD, stateHistory.statesAgo(20));
    }
    public double getWheelPosition(SD wheelAngleSD, RobotState state){
        // Takes a spark. Returns the last recorded pos of that spark/wheel
        return Robot.gearShiftSubsystem.getCurrentGearRatio() * state.get(wheelAngleSD) * WHEEL_RADIUS;
    }

    public RobotState nextPosition(double x, double y, double theta, ArrayList<Double> wheelDistances){
        // Works for all forms of drive where the displacement is the average of the movement vectors over the wheels
        double newTheta = pigeon.getAngle();
        RobotState robotState = new RobotState();
        
        if (x != 0 || robotState.get(SD.X) != 0){
            System.out.println("new x:" + x);
            System.out.println("rs x: " + robotState.get(SD.X));
        }
        double avgTheta = (theta + newTheta)/2;
        for(double wheelDistance : wheelDistances){
            if (wheelDistance != 0){
                //System.out.println("x change: " + wheelChangeInfo.rotationChange * Math.cos(avgTheta + wheelChangeInfo.angle) / wheelAmount);
            }
            x += wheelDistance * Math.cos(avgTheta) / wheelDistances.size();
            y += wheelDistance * Math.sin(avgTheta) / wheelDistances.size();
        }
        robotState.set(SD.X, x);
        robotState.set(SD.Y, y);
        robotState.set(SD.Angle, newTheta);
        robotState.set(SD.GearShiftSolenoid, 0.0);
        return robotState;
    }
 
    public RobotState nextPosTankPigeon(double x, double y, double theta, double leftChange, double rightChange) {
        // This assumes tank drive and you want to use the pigeon for calculating your angle
        // Takes a pos (x,y,theta), a left side Δx and a right side Δx and returns an x,y,theta array
        ArrayList<Double> allChangeInfo = new ArrayList<>();
        if (x != 0){
            System.out.println("x: " + x);
        }
        allChangeInfo.add(leftChange);
        allChangeInfo.add(rightChange); // the zeros represent that they aren't turned
        return nextPosition(x,y,theta,allChangeInfo);
    }

    @Override
    public RobotState updateState(RobotStateHistory pastStates){
        RobotState state = pastStates.currentState();
       // Robot.delayedPrint("left wheel angle: " + Robot.get(SD.LeftWheelAngle));
       // Robot.delayedPrint("" + wheelRotationChange(SD.LeftWheelAngle, pastStates));
       if (Robot.get(SD.X) != 0){
           System.out.println("robot: " + Robot.get(SD.X));
           System.out.println("state: " + state.get(SD.X));
       }
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