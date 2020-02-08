package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciTalonSpeedCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.StateInfo;
import frc.robot.Robot;
import frc.robot.stateEstimation.interfaces.Model;
import frc.robot.robotState.RobotState.SD;

import java.util.Optional;
import java.util.ArrayList;

public class SciTalon extends TalonSRX implements RobotStateUpdater {

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
    public static final int DEFAULT_FREE_CURRNET_LIMT = 40;
    public static final int DEFAULT_SPIKE_CURRENT_LIMIT = 100;
    public static final double DEFAULT_SPIKE_MAX_TIME = 0.1;
    public static final double CHOP_CYCLE_DURATION = 0.05;
    public static final int DEFAULT_CHOP_CYCLES = (int) (DEFAULT_SPIKE_MAX_TIME / CHOP_CYCLE_DURATION);
    public static final double ENC_TICKS_PER_REV = 4096; // For talons
    public double decrementSnapSpeed = .3;
    private double positionConversionFactor;
    private double velocityConversionFactor;

    public SciTalon(int port) {
        this(port, 1);
    }

    public SciTalon(int port, double gearRatio) {
        super(port);
        super.configContinuousCurrentLimit(10, 0);
        super.configPeakCurrentLimit(15, 0);
        super.configPeakCurrentDuration(100, 0);
        super.enableCurrentLimit(true);
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
        Robot.addRobotStateUpdater(this);
    }

    public String getDeviceName() {return "Talon " + super.getDeviceID();}
    
    public double getGearRatio()      {return this.gearRatio;} 
    public int getMovementPrecision() {return this.movmentPrecision;}
    
    public void setMovementPrecision(int movementPrecision){this.movmentPrecision = movementPrecision;}

    public void diminishSnap(){this.diminishSnap = true;}
    public void ignoreSnap()  {this.diminishSnap = false;}

    public void setPositionConversionFactor () { this.gearRatio *= 2 * Math.PI / ENC_TICKS_PER_REV; }
    public void setVelocityConversionFactor () { this.velocityConversionFactor = this.positionConversionFactor/ Utils.SECONDS_PER_MINUTE; }

    public void setGearRatio(double gearRatio) {
        double currentAngle = determineWheelAngle();
        this.gearRatio = gearRatio;
        setPositionConversionFactor();
        // I don't know if we have to multiply this by getPositionConversionFactor()
        setVelocityConversionFactor();
        // W/o this when we changed gear ratios, it would weirdly change the angle
        // For instance, if you have a gear ratio of 1/5 and your getPosition() is 20 radians
        //  and then you change the gear ratio to 1/10, your getPosition() will go to 10 radians
        setWheelAngle(currentAngle);
    }

    public void setWheelAngle(double angle) {
        super.getSensorCollection().setQuadraturePosition((int) (angle / positionConversionFactor), 0);
    }

    private double determineWheelAngle() {return super.getSensorCollection().getQuadraturePosition() * this.positionConversionFactor;}
    private double determineVelocity()   {return super.getSensorCollection().getQuadratureVelocity() * this.velocityConversionFactor;}

    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;}
    public boolean atGoal() {return this.goalSpeed == super.getMotorOutputPercent();}
    public void instantSet() {
        double limitedInput = Utils.limitChange(super.getMotorOutputPercent(), this.goalSpeed, this.currentMaxJerk);
        double input = this.diminishSnap ? diminishSnap(limitedInput) : limitedInput;
        super.set(ControlMode.PercentOutput, limitedInput);
        checkWarningStatus(input, super.getMotorOutputPercent());
    }

    public void checkWarningStatus(double input, double realOutput){
        if (!Utils.inRange(input, super.getMotorOutputPercent(), TOLERABLE_DIFFERENCE)) {
            // (new TalonDelayWarningCommand(this, input)).start();
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

    public void set(double speed, double maxJerk){
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        // Set will call this command, which will continue to call instantSet
        // InstantSet will only set the value of the motor to the correct value if it is within maxJerk
        (new SciTalonSpeedCommand(this, this.commandNumber)).start();
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
        Robot.optionalSet(this.wheelAngleSD, determineWheelAngle());
        Robot.optionalSet(this.velocitySD,   determineVelocity());
        Robot.optionalSet(this.valueSD,      super.getMotorOutputPercent());
        Robot.optionalSet(this.currentSD,    super.getSupplyCurrent());
        this.accelModel.updateRobotState();
        this.jerkModel .updateRobotState();
        this.snapModel .updateRobotState();
        if(this.printValues){
            DelayedPrinter.print("talon " + super.getDeviceID() + " value: " + super.getMotorOutputPercent());
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

    public SciTalon talon;
    public AccelModel(SciTalon talon) {this.talon = talon;}
    @Override
    public void updateRobotState() {
        if(this.talon.accelSD.isPresent()) {
            Robot.optionalSet(this.talon.accelSD, StateInfo.getRateOfChange(this.talon.velocitySD.get(), talon.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.talon.accelSD);}
}

class JerkModel implements Model {

    public SciTalon talon;
    public JerkModel(SciTalon talon) {this.talon = talon;}
    @Override
    public void updateRobotState() {
        if(this.talon.jerkSD.isPresent()) {
            Robot.optionalSet(this.talon.jerkSD, StateInfo.getRateOfChange(this.talon.accelSD.get(), talon.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.talon.jerkSD);}

}

class SnapModel implements Model {

    public SciTalon talon;
    public SnapModel(SciTalon talon) {this.talon = talon;}
    @Override
    public void updateRobotState() {
        if(this.talon.snapSD.isPresent()) {
            Robot.optionalSet(this.talon.snapSD,StateInfo.getRateOfChange(this.talon.jerkSD.get(), talon.getMovementPrecision()));
        }
    }
    @Override public Iterable<SD> getSDs() {return Utils.optionalInitArrayList(this.talon.snapSD);}

}