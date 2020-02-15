package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.Optional;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.commands.SparkDelayWarningCommand;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.stateEstimation.interfaces.Model;

public class SciSpark extends CANSparkMax implements RobotStateUpdater {

    public double goalSpeed;
    public double currentMaxJerk;
    public static final double DEFAULT_MAX_JERK = 0.15;
    private int movmentPrecision = 3;
    private double gearRatio;
    private Model accelModel, jerkModel, snapModel;
    public Optional<SD> wheelAngleSD, velocitySD, accelSD, jerkSD, snapSD, valueSD, currentSD;
    private int commandNumber;
    private boolean printValues;
    private boolean diminishSnap = false;
    public static final double TOLERABLE_DIFFERENCE = 0.01;
    public static final int DEFAULT_STALL_CURRENT_LIMIT = 40;
    public static final int DEFAULT_FREE_CURRENT_LIMT = 40;
    public static final int DEFAULT_SPIKE_CURRENT_LIMIT = 100;
    public static final double DEFAULT_SPIKE_MAX_TIME = 0.1;
    public static final double CHOP_CYCLE_DURATION = 0.05;
    public static final int DEFAULT_CHOP_CYCLES = (int) (DEFAULT_SPIKE_MAX_TIME / CHOP_CYCLE_DURATION);
    public double decrementSnapSpeed = .3;

    public SciSpark(int port) {
        this(port, 1);
    }

    public SciSpark(int port, double gearRatio) {
        super(port, MotorType.kBrushless);
        super.setSmartCurrentLimit(DEFAULT_STALL_CURRENT_LIMIT, DEFAULT_FREE_CURRENT_LIMT);
        super.setSecondaryCurrentLimit(DEFAULT_SPIKE_MAX_TIME, DEFAULT_CHOP_CYCLES);
        this.goalSpeed = 0;
        this.currentMaxJerk = DEFAULT_MAX_JERK;
        this.commandNumber = 0;
        this.wheelAngleSD = Optional.empty();
        this.velocitySD   = Optional.empty();
        this.accelSD      = Optional.empty();
        this.jerkSD       = Optional.empty();
        this.snapSD       = Optional.empty();
        this.valueSD      = Optional.empty();
        this.currentSD    = Optional.empty();
        this.printValues = false;
        this.accelModel = new AccelModel(this);
        this.jerkModel  = new JerkModel(this);
        this.snapModel  = new SnapModel(this);
        setWheelAngle(0);
        setGearRatio(gearRatio);
        Robot.addRobotStateUpdater(this);
    }

    public String getDeviceName() {return "Spark " + super.getDeviceId();}

    public double getGearRatio()      {return this.gearRatio;}
    public int getMovementPrecision() {return this.movmentPrecision;}
    
    public void setMovementPrecision(int movementPrecision){this.movmentPrecision = movementPrecision;}
    
    public void diminishSnap(){this.diminishSnap = true;}
    public void ignoreSnap()  {this.diminishSnap = false;} 

    public void setGearRatio(double gearRatio) {
        double currentAngle = determineWheelAngle();
        this.gearRatio = gearRatio;
        super.getEncoder().setPositionConversionFactor(this.gearRatio * 2 * Math.PI);
        // I don't know if we have to multiply this by getPositionConversionFactor()
        super.getEncoder().setVelocityConversionFactor(super.getEncoder().getPositionConversionFactor() / Utils.SECONDS_PER_MINUTE); 
        // W/o this when we changed gear ratios, it would weirdly change the angle
        // For instance, if you have a gear ratio of 1/5 and your getPosition() is 20 radians
        //  and then you change the gear ratio to 1/10, your getPosition() will go to 10 radians
        setWheelAngle(currentAngle);
    }

    public void setWheelAngle(double angle) {
        super.getEncoder().setPosition(angle);
    }

    private double determineWheelAngle() {return super.getEncoder().getPosition();}
    private double determineVelocity()   {return super.getEncoder().getVelocity();}

    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;} 
    public boolean atGoal() {return this.goalSpeed == super.get();}
    public void instantSet() {
        double limitedInput = Utils.limitChange(super.get(), this.goalSpeed, this.currentMaxJerk);
        double input = this.diminishSnap ? diminishSnap(limitedInput) : limitedInput;
        super.set(input);
        checkWarningStatus(input, super.get());
    }

    public void checkWarningStatus(double input, double realOutput){
        if (!Utils.inRange(input, super.get(), TOLERABLE_DIFFERENCE)) {
            (new SparkDelayWarningCommand(this, input)).start();
        }   
    }

    private double snapDecrementer(double x){
        double sign = Math.signum(x);
        x = Math.abs(x);
        return sign * Math.pow(x, 1/(1 + Math.log(Math.sqrt(x) + 1)*this.decrementSnapSpeed));
    }

    private double diminishSnap(double input){
        double lastJerk = StateInfo.getDifference(Robot.stateHistory, this.valueSD.get(), 1);
        double currentJerk = input - Robot.get(this.valueSD.get());
        double snap = currentJerk - lastJerk;
        double newSnap = snapDecrementer(snap);
        double newJerk = lastJerk + newSnap;
        return Robot.get(this.valueSD.get()) + newJerk;
    }

    public void set(double speed, double maxJerk) {
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        // Set will call this command, which will continue to call instantSet
        // InstantSet will only set the value of the motor to the correct value if it is within maxJerk
        (new SciSparkSpeedCommand(this, this.commandNumber)).start();
    }
    public void set(double speed) {set(speed, DEFAULT_MAX_JERK);}

    public void printDebuggingInfo() {
        System.out.println("Debugging info:");
        if (this.wheelAngleSD.isPresent()) {
            double positionChange = StateInfo.getFullDifference(this.wheelAngleSD.get());
            System.out.println("Position Change: " + positionChange);
            System.out.println("Is significant?: " + (Math.abs(positionChange) > Utils.EPSILON));
            System.out.println("All Positions: " + Robot.stateHistory.getFullSDData(this.wheelAngleSD.get()));
        }
        if (this.currentSD.isPresent()) {
            System.out.println("Current: " + Robot.get(this.currentSD.get()));
        }
        if (this.valueSD.isPresent()) {
           System.out.println("All Values: " + Robot.stateHistory.getFullSDData(this.valueSD.get()));
        }
    }

    public void printValues()    {this.printValues = true;}
    public void dontPrintValues(){this.printValues = false;}

    public void updateRobotState(){
        if (getDeviceId() < 20){
            Robot.optionalSet(this.wheelAngleSD, determineWheelAngle());
            Robot.optionalSet(this.velocitySD,   determineVelocity());
            Robot.optionalSet(this.valueSD,      super.get());
            Robot.optionalSet(this.currentSD,    super.getOutputCurrent());
            this.accelModel.updateRobotState();
            this.jerkModel .updateRobotState();
            this.snapModel .updateRobotState();
            if(this.printValues){
                DelayedPrinter.print("Spark " + super.getDeviceId() + " value: " + super.get());
            }
        }
    }

    public void assignWheelAngleSD(SD wheelAngleSD) {this.wheelAngleSD = Optional.of(wheelAngleSD); Robot.set(wheelAngleSD, 0);}
    public void assignVelocitySD  (SD velocitySD)   {this.velocitySD   = Optional.of(velocitySD);   Robot.set(velocitySD  , 0);}
    public void assignAccelD      (SD accelSD)      {this.accelSD      = Optional.of(accelSD);      Robot.set(accelSD     , 0);}
    public void assignJerkSD      (SD jerkSD)       {this.jerkSD       = Optional.of(jerkSD);       Robot.set(jerkSD      , 0);}
    public void assignSnapSD      (SD snapSD)       {this.snapSD       = Optional.of(snapSD);       Robot.set(snapSD      , 0);}
    public void assignValueSD     (SD valueSD)      {this.valueSD      = Optional.of(valueSD);      Robot.set(valueSD     , 0);}
    public void assignCurrentSD   (SD currentSD)    {this.currentSD    = Optional.of(currentSD);    Robot.set(currentSD   , 0);}

    public ArrayList<SD> getAllSDs(){
        return Utils.optionalInitArrayList(wheelAngleSD, velocitySD, accelSD, jerkSD, snapSD, valueSD, currentSD);
    }

    public void logAllSDs(){
        for(SD sd : getAllSDs()){Robot.addSDToLog(sd);}
    }

}

class AccelModel implements Model {

    public SciSpark spark;
    public AccelModel(SciSpark spark) {this.spark = spark;}
    @Override
    public void updateRobotState() {
        if(this.spark.accelSD.isPresent()) {
            Robot.optionalSet(this.spark.accelSD, StateInfo.getRateOfChange(this.spark.velocitySD.get(), spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.spark.accelSD);}
}

class JerkModel implements Model {

    public SciSpark spark;
    public JerkModel(SciSpark spark) {this.spark = spark;}
    @Override
    public void updateRobotState() {
        if(this.spark.jerkSD.isPresent()) {
            Robot.optionalSet(this.spark.jerkSD, StateInfo.getRateOfChange(this.spark.accelSD.get(), spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.spark.jerkSD);}

}

class SnapModel implements Model {

    public SciSpark spark;
    public SnapModel(SciSpark spark) {this.spark = spark;}
    @Override
    public void updateRobotState() {
        if(this.spark.snapSD.isPresent()) {
            Robot.optionalSet(this.spark.snapSD,StateInfo.getRateOfChange(this.spark.jerkSD.get(), spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.spark.snapSD);}

}