package frc.robot.stateEstimation;

import frc.robot.Robot;

import java.util.ArrayList;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.RobotState;
import frc.robot.RobotState.RS;
import frc.robot.helpers.Pigeon;
import frc.robot.logging.Logger.DefaultValue;

public class EncoderLocalization implements Updater, PositionModel {

    public static final double WHEEL_RADIUS = Utils.inchesToMeters(3);
    public static final double ROBOT_RADIUS = Utils.inchesToMeters(15.945); // Half the distance from wheel to wheel

    public static final double ORIGINAL_ANGLE = Math.PI/2, ORIGINAL_X = 0, ORIGINAL_Y = 0;
    public static final double INTERVAL_LENGTH = .02; // Seconds between each tick for commands
    private final String FILENAME = "RobotPosition.java";

    private ArrayList<CANSparkMax> sparks;
    private Pigeon pigeon;
    private TalonSRX pigeonTalon;

    public EncoderLocalization(){

        this.pigeonTalon = new TalonSRX(PortMap.PIGEON_TALON);
        this.pigeon      = new Pigeon(pigeonTalon);

        this.sparks         = new ArrayList<>();

        keepTrackOfWheel(Robot.driveSubsystem.lf);
        keepTrackOfWheel(Robot.driveSubsystem.rf);
    }

    public TalonSRX[] getTalons() {
        return new TalonSRX[]{this.pigeonTalon};
    }

    public Pigeon getPigeon(){return this.pigeon;}
    
    public void keepTrackOfWheel(CANSparkMax spark) {
        // this allows the code to easily keep track of the position of motors for the
        // chassis
        this.sparks.add(spark);
    }

    public double calculateWheelPosition(CANSparkMax motor) {
        // Returns the encoder position of a spark
        // TODO: Should change to alternate low gear/high gear with whatever it is
        return Robot.gearShiftSubsystem.getCurrentGearRatio() * Robot.encoderSubsystem.getSparkAngle(motor) * WHEEL_RADIUS;
    }
    
    public double wheelRotationChange(CANSparkMax spark, ArrayList<RobotState> states){
        return getWheelPosition(spark, states, 0) - getWheelPosition(spark, states, 1);
    }
    
    public double getWheelPosition(CANSparkMax spark, ArrayList<RobotState> robotStates, int ticksAgo)  {
        return getWheelPosition(spark, robotStates.get(ticksAgo));
    }
    public double getWheelPosition(CANSparkMax spark, RobotState state){
        // Takes a spark. Returns the last recorded pos of that spark/wheel
        return state.get(Robot.driveSubsystem.sparkToWheelRS.get(spark));
    }

    public Hashtable<String,Double> newWheelChangeInfo(double rotationChange, double angle){
        return new Hashtable<String,Double>(){{
            put("rotationChange", rotationChange);
            put("angle", angle);
        }};
    }

    public RobotState nextPosition(double x, double y, double theta, ArrayList<Hashtable<String,Double>> allChangeInfo){
        // Works for all forms of drive where the displacement is the average of the movement vectors over the wheels
        double newTheta = pigeon.getAngle();
        double avgTheta = (theta + newTheta)/2;
        int wheelAmount = allChangeInfo.size();
        for(Hashtable<String,Double> wheelChangeInfo : allChangeInfo){
            x += wheelChangeInfo.get("rotationChange") * Math.cos(avgTheta + wheelChangeInfo.get("angle")) / wheelAmount;
            y += wheelChangeInfo.get("rotationChange") * Math.sin(avgTheta + wheelChangeInfo.get("angle")) / wheelAmount;
        }
        RobotState state = new RobotState();
        state.set(RS.X, x);
        state.set(RS.Y, y);
        state.set(RS.Angle, newTheta);
        return state;
    }
 
    public RobotState nextPosTankPigeon(double x, double y, double theta, double leftChange, double rightChange) {
        // This assumes tank drive and you want to use the pigeon for calculating your angle
        // Takes a pos (x,y,theta), a left side Δx and a right side Δx and returns an x,y,theta array
        ArrayList<Hashtable<String,Double>> allChangeInfo = new ArrayList<Hashtable<String,Double>>();
        allChangeInfo.add(newWheelChangeInfo(leftChange,0));
        allChangeInfo.add(newWheelChangeInfo(rightChange, 0)); // the zeros represent that they aren't turned
        return nextPosition(x,y,theta,allChangeInfo);
    }

    public RobotState updateState(ArrayList<RobotState> pastStates){
        RobotState newPosition = 
            nextPosTankPigeon(Robot.get(RS.X), Robot.get(RS.Y), Robot.get(RS.Angle), 
                wheelRotationChange(Robot.driveSubsystem.lm, pastStates), 
                wheelRotationChange(Robot.driveSubsystem.rm, pastStates));
        return pastStates.get(0).incorporateIntoNew(newPosition); 
    }

    public void updatePosition(){
        Robot.robotStates.set(0, updateState(Robot.robotStates));
    }
    
	public void periodicLog(){
        Robot.logger.addData(FILENAME, "robot X",     Robot.get(RS.X),     DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "robot y",     Robot.get(RS.Y),     DefaultValue.Previous);
        Robot.logger.addData(FILENAME, "robot angle", Robot.get(RS.Angle), DefaultValue.Previous);
	}

    public void printPosition(){
        System.out.println("X: " + Robot.get(RS.X));
        System.out.println("Y: " + Robot.get(RS.Y));
        System.out.println("Angle: " + Math.toDegrees(Robot.get(RS.Angle)));
    }
}