package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciTalonSpeedCommand;
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
    private SciUtils<SciTalonSD> sciUtils; 
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
        this.sciUtils = new SciUtils<>(this);
        this.sdMap = new HashMap<>();
        Robot.addRobotStateUpdater(this);
    }
    @Override
    public HashMap<SciTalonSD, SD> getSDMap(){return this.sdMap;}
    @Override 
    public SciUtils<SciTalonSD> getSciUtils(){return this.sciUtils;}
    @Override 
    public String getDeviceName(){return "Talon " + super.getDeviceID();}

    @Override
    public void assignSD(SciTalonSD talonSD, SD sd) {
        this.sciUtils.assignSD(talonSD, sd);
    }


    public double getGearRatio() {return this.gearRatio;}
    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;}
    public boolean atGoal() {return this.goalSpeed == super.getMotorOutputPercent();}
    public void instantSet() {
        double limitedInput = Utils.limitChange(super.getMotorOutputPercent(), this.goalSpeed, this.currentMaxJerk);
        super.set(ControlMode.PercentOutput, limitedInput);
        if (limitedInput != super.getMotorOutputPercent()) {
            String warning = "WARNING: Talon " + super.getDeviceID() + " was set to " + limitedInput
                    + " but still has a value of " + super.getMotorOutputPercent();
            System.out.println(warning);
            System.out.println(warning);
            printDebuggingInfo();
        }
    }

    public void printDebuggingInfo() {
        System.out.println("Debugging info:");
        this.sciUtils.printAllData();
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
        this.sciUtils.sciSet(SciTalonSD.ValueSD,   super.getMotorOutputPercent());
        this.sciUtils.sciSet(SciTalonSD.CurrentSD, super.getSupplyCurrent());
        if(this.printValues){
            DelayedPrinter.print("Talon " + super.getDeviceID() + " value: " + super.getMotorOutputPercent());
        }
    }
}