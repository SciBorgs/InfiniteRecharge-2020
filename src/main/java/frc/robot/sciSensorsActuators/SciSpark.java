package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;

public class SciSpark extends CANSparkMax implements RobotStateUpdater {

    public double goalSpeed;
    public double currentMaxJerk;
    public final static double DEFAULT_MAX_JERK = 0.1;
    private double gearRatio;
    public Optional<SD> wheelAngleSD, valueSD, currentSD;
    private int commandNumber;
    private boolean printValues;
    public static final int DEFAULT_STALL_CURRENT_LIMIT = 40;
    public static final int DEFAULT_FREE_CURRNET_LIMT = 40;
    public static final int DEFAULT_SPIKE_CURRENT_LIMIT = 100;
    public static final double DEFAULT_SPIKE_MAX_TIME = 0.1;
    public static final double CHOP_CYCLE_DURATION = 0.05;
    public static final int DEFAULT_CHOP_CYCLES = (int) (DEFAULT_SPIKE_MAX_TIME / CHOP_CYCLE_DURATION);

    public SciSpark(int port) {
        this(port, 1);
    }

    public SciSpark(int port, double gearRatio) {
        super(port, MotorType.kBrushless);
        super.setSmartCurrentLimit(DEFAULT_STALL_CURRENT_LIMIT, DEFAULT_FREE_CURRNET_LIMT);
        super.setSecondaryCurrentLimit(DEFAULT_SPIKE_MAX_TIME, DEFAULT_CHOP_CYCLES);
        this.goalSpeed = 0;
        this.currentMaxJerk = DEFAULT_MAX_JERK;
        this.commandNumber = 0;
        this.wheelAngleSD = Optional.empty();
        this.valueSD      = Optional.empty();
        this.currentSD    = Optional.empty();
        this.printValues = false;
        setWheelAngle(0);
        setGearRatio(gearRatio);
        Robot.addRobotStateUpdater(this);
    }

    public double getGearRatio() {return this.gearRatio;}

    public void setGearRatio(double gearRatio) {
        double currentAngle = getWheelAngle();
        this.gearRatio = gearRatio;
        super.getEncoder().setPositionConversionFactor(this.gearRatio * 2 * Math.PI);
        // W/o this when we changed gear ratios, it would weirdly change the angle
        // For instance, if you have a gear ratio of 1/5 and your getPosition() is 20 radians
        //  and then you change the gear ratio to 1/10, your getPosition() will go to 10 radians
        setWheelAngle(currentAngle);
    }

    public void setWheelAngle(double angle) {
        super.getEncoder().setPosition(angle / super.getEncoder().getPositionConversionFactor());
    }

    public double getWheelAngle() {
        return super.getEncoder().getPosition();
    }

    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;} 
    public boolean atGoal() {return this.goalSpeed == super.get();}
    public void instantSet() {
        double limitedInput = Utils.limitChange(super.get(), this.goalSpeed, this.currentMaxJerk);
        super.set(limitedInput);
        if (limitedInput != super.get()) {
            String warning = "WARNING: Spark " + super.getDeviceId() + " was set to " + limitedInput
                    + " but still has a value of " + super.get();
            System.out.println(warning);
            System.out.println(warning);
            printDebuggingInfo();
        }
    }

    public void set(double speed, double maxJerk) {
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        // Set will call this command, which will continue to call instantSet
        // InstantSet will only set the value of the motor to the correct value if it is within maxJerk
        CommandScheduler.getInstance().schedule(new SciSparkSpeedCommand(this, this.commandNumber));
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
        Robot.optionalSet(this.wheelAngleSD, getWheelAngle());
        Robot.optionalSet(this.valueSD,      super.get());
        Robot.optionalSet(this.currentSD,    super.getOutputCurrent());
        if(this.printValues){
            DelayedPrinter.print("Spark " + super.getDeviceId() + " value: " + super.get());
        }
    }

    public void assignWheelAngleSD(SD wheelAngleSD) {this.wheelAngleSD = Optional.of(wheelAngleSD);}
    public void assignValueSD     (SD valueSD)      {this.valueSD      = Optional.of(valueSD);}
    public void assignCurrentSD   (SD currentSD)    {this.currentSD    = Optional.of(currentSD);}
    public void assignAll(SD wheelAngleSD, SD valueSD, SD currentSD){
        assignWheelAngleSD(wheelAngleSD);
        assignValueSD(valueSD);
        assignCurrentSD(currentSD);
    }
}