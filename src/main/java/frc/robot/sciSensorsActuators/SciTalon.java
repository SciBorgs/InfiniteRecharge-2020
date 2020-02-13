package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciTalonSpeedCommand;
import frc.robot.commands.generalCommands.TalonDelayWarningCommand;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciTalon.SciTalonSD;

import java.util.HashMap;
import java.util.Optional;

public class SciTalon extends TalonSRX implements RobotStateUpdater, SciSensorActuator<SciTalonSD> {

    public static enum SciTalonSD {ValueSD, CurrentSD;}
    public static final double DEFAULT_MAX_JERK = 0.1;
    public HashMap<SciTalonSD, SD> sdMap;
    public static final double TOLERABLE_DIFFERENCE = 0.01;
    private int commandNumber;
    public double goalSpeed;
    public double currentMaxJerk;
    public double gearRatio;
    public boolean printValues;

    public SciTalon(int port) {
        this(port, 1);
    }

    public SciTalon(int port, double gearRatio) {
        super(port);
        this.goalSpeed = 0;
        this.currentMaxJerk = 0;
        this.commandNumber = 0;
        this.printValues = false;
        this.sdMap = new HashMap<>();
        Robot.addRobotStateUpdater(this);
    }
    @Override
    public HashMap<SciTalonSD, SD> getSDMap(){return this.sdMap;}
    @Override 
    public String getDeviceName(){return "Talon " + super.getDeviceID();}


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
        sciSet(SciTalonSD.ValueSD,   super.getMotorOutputPercent());
        sciSet(SciTalonSD.CurrentSD, super.getSupplyCurrent());
        if(this.printValues){
            DelayedPrinter.print("Talon " + super.getDeviceID() + " value: " + super.getMotorOutputPercent());
        }
    }
}