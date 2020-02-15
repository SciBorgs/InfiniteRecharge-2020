package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciTalonSpeedCommand;
import frc.robot.commands.TalonDelayWarningCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;


import java.util.Optional;

public class SciTalon extends TalonSRX implements RobotStateUpdater {

    public static final double DEFAULT_MAX_JERK = 0.1;
    public static final double TOLERABLE_DIFFERENCE = 0.01;
    private int commandNumber;
    public double goalSpeed;
    public double currentMaxJerk;
    public double gearRatio;
    public Optional<SD> valueSD, currentSD;
    public boolean printValues;

    public SciTalon(int port) {
        this(port, 1);
    }

    public SciTalon(int port, double gearRatio) {
        super(port);
        this.goalSpeed = 0;
        this.currentMaxJerk = 0;
        this.commandNumber = 0;
        this.valueSD   = Optional.empty();
        this.currentSD = Optional.empty();
        this.printValues = false;
        Robot.addRobotStateUpdater(this);
    }


    public double getGearRatio() {return this.gearRatio;}
    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;}
    public boolean atGoal() {return this.goalSpeed == super.getMotorOutputPercent();}
    public void instantSet() {
        double limitedInput = Utils.limitChange(super.getMotorOutputPercent(), this.goalSpeed, this.currentMaxJerk);
        super.set(ControlMode.PercentOutput, limitedInput);
        checkWarningStatus(limitedInput,super.getMotorOutputPercent());
    }
    public void checkWarningStatus(double input, double realOutput){
        if (!Utils.inRange(input, super.getMotorOutputPercent(), TOLERABLE_DIFFERENCE)) {
            (new TalonDelayWarningCommand(this, input)).start();
        }   
    }

    public void printDebuggingInfo() {
        System.out.println("Debugging info:");
        if (this.currentSD.isPresent()) {
            System.out.println("Current: " + Robot.get(this.currentSD.get()));
        }
        if (this.valueSD.isPresent()) {
            System.out.println("All Values: " + Robot.stateHistory.getFullSDData(this.valueSD.get()));
        }
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

    public void printValues()    {this.printValues = true;}
    public void dontPrintValues(){this.printValues = false;}

    public void updateRobotState(){
        // Robot.optionalSet(this.valueSD,   super.getMotorOutputPercent());
        // Robot.optionalSet(this.currentSD, super.getSupplyCurrent());
        // if(this.printValues){
        //     DelayedPrinter.print("Talon " + super.getDeviceID() + " value: " + super.getMotorOutputPercent());
        // }
    }

    public void assignValueSD  (SD valueSD)   {this.valueSD   = Optional.of(valueSD);}
    public void assignCurrentSD(SD currentSD) {this.currentSD = Optional.of(currentSD);}
}