package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.commands.generalCommands.SparkDelayWarningCommand;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark.SciSparkSD;
import frc.robot.stateEstimation.interfaces.Model;

public class SciSpark extends CANSparkMax implements RobotStateUpdater, SciSensorActuator<SciSparkSD> {

    public static enum SciSparkSD {WheelAngle, Velocity, Accel, Jerk, Snap, Value, Current}
    public double goalSpeed;
    public double currentMaxJerk;
    public static final double DEFAULT_MAX_JERK = 0.15;
    private int movmentPrecision = 3;
    private double gearRatio;
    private Model accelModel, jerkModel, snapModel;
    private HashMap<SciSparkSD, SD> sdMap;
    private int commandNumber;
    private boolean printValues;
    private boolean diminishSnap = false;
    private boolean isSetting;
    public static final double TOLERABLE_DIFFERENCE = 0.01;
    public static final int DEFAULT_STALL_CURRENT_LIMIT = 40;
    public static final int DEFAULT_FREE_CURRENT_LIMT = 40;
    public static final int DEFAULT_SPIKE_CURRENT_LIMIT = 100;
    public static final double DEFAULT_SPIKE_MAX_TIME = 0.1;
    public static final double CHOP_CYCLE_DURATION = 0.05;
    public static final int DEFAULT_CHOP_CYCLES = (int) (DEFAULT_SPIKE_MAX_TIME / CHOP_CYCLE_DURATION);
    public double decrementSnapSpeed = .3;

    @Override
    public boolean ignore(){return sciIgnore(super.getDeviceId());}

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
        this.sdMap = new HashMap<>();
        this.printValues = false;
        this.accelModel = new AccelModel(this);
        this.jerkModel  = new JerkModel (this);
        this.snapModel  = new SnapModel (this);
        setWheelAngle(0);
        setGearRatio(gearRatio);
        automateStateUpdating();
    }

    @Override
    public HashMap<SciSparkSD, SD> getSDMap(){return this.sdMap;}
    @Override
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
        if(ignore() && !this.isSetting){return;}
        this.isSetting = true;
        double limitedInput = Utils.limitChange(super.get(), this.goalSpeed, this.currentMaxJerk);
        double input = this.diminishSnap ? diminishSnap(limitedInput) : limitedInput;
        super.set(input);
        checkWarningStatus(input, super.get());
        this.isSetting = false;
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
        double lastJerk = StateInfo.getDifference(Robot.stateHistory, sdOf(SciSparkSD.Value), 1);
        double currentJerk = input - sciGet(SciSparkSD.Value);
        double snap = currentJerk - lastJerk;
        double newSnap = snapDecrementer(snap);
        double newJerk = lastJerk + newSnap;
        return sciGet(SciSparkSD.Value) + newJerk;
    }

    public void set(double speed, double maxJerk) {
        if(ignore()){return;}
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        // Set will call this command, which will continue to call instantSet
        // InstantSet will only set the value of the motor to the correct value if it is within maxJerk
        (new SciSparkSpeedCommand(this, this.commandNumber)).start();
    }
    public void set(double speed) {set(speed, DEFAULT_MAX_JERK);}

    public void printValues()    {this.printValues = true;}
    public void dontPrintValues(){this.printValues = false;}

    public void updateRobotState(){
        sciSet(SciSparkSD.WheelAngle, determineWheelAngle());
        sciSet(SciSparkSD.Velocity,   determineVelocity());
        sciSet(SciSparkSD.Value,      super.get());
        sciSet(SciSparkSD.Current,    super.getOutputCurrent());
        this.accelModel.updateRobotState();
        this.jerkModel .updateRobotState();
        this.snapModel .updateRobotState();
        if(this.printValues){
            DelayedPrinter.print("Spark " + super.getDeviceId() + " value: " + super.get());
        }
    }

}

class AccelModel implements Model {

    public SciSpark spark;
    public AccelModel(SciSpark spark) {
        this.spark = spark;
    }
    @Override
    public void updateRobotState() {
        if(this.spark.isTracked(SciSparkSD.Accel)) {
            this.spark.sciSet(SciSparkSD.Accel, 
                StateInfo.getRateOfChange(this.spark.sdOf(SciSparkSD.Velocity), this.spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return this.spark.getSDMap().values();}
}

class JerkModel implements Model {

    public SciSpark spark;
    public JerkModel(SciSpark spark) {
        this.spark = spark;
    }
    @Override
    public void updateRobotState() {
        if(this.spark.isTracked(SciSparkSD.Jerk)) {
            this.spark.sciSet(SciSparkSD.Jerk, 
                StateInfo.getRateOfChange(this.spark.sdOf(SciSparkSD.Accel), this.spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return this.spark.getSDMap().values();}
}

class SnapModel implements Model {

    public SciSpark spark;
    public SnapModel(SciSpark spark) {
        this.spark = spark;
    }
    @Override
    public void updateRobotState() {
        if(this.spark.isTracked(SciSparkSD.Snap)) {
            this.spark.sciSet(SciSparkSD.Snap, 
                StateInfo.getRateOfChange(this.spark.sdOf(SciSparkSD.Jerk), this.spark.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return this.spark.getSDMap().values();}
}