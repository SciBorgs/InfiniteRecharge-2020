package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import java.util.Optional;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;

public class SciSpark extends CANSparkMax {

    public double goalSpeed;
    public double currentMaxJerk;
    public final static double DEFAULT_MAX_JERK = 0.1;
    private double gearRatio;
    public Optional<SD> wheelAngleSD, valueSD, currentSD;
    private int commandNumber;


    public SciSpark(int port) {
        this(port, 1);
    }

    public SciSpark(int port, double gearRatio) {
        super(port, MotorType.kBrushless);
        this.goalSpeed = 0;
        this.currentMaxJerk = DEFAULT_MAX_JERK;
        setWheelAngle(0);
        setGearRatio(gearRatio);
        this.commandNumber = 0;
    }

    public boolean isCurrentCommandNumber(int n){ return n == this.commandNumber; } 

    public double getGearRatio() {
        return this.gearRatio;
    }

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

    public void instantSet(double speed, double maxJerk) {
        super.set(Utils.limitChange(super.get(), speed, maxJerk));
    }

    public void set(double speed, double maxJerk) {
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        double limitedInput = Utils.limitChange(super.get(), speed, maxJerk);
        if (limitedInput != super.get()) {
            String warning = "WARNING: Spark " + super.getDeviceId() + " was set to " + limitedInput + " but still has a value of " + super.get();
            System.out.println(warning);
            System.out.println(warning);
            System.out.println(warning);
            System.out.println(warning);
            System.out.println("Debugging info:");
            if (wheelAngleSD.isPresent()) {
                double positionChange = StateInfo.getFullDifference(this.wheelAngleSD.get());
                System.out.println("Position Change: " + positionChange);
                System.out.println("Is significant?: " + (Math.abs(positionChange) > Utils.EPSILON));
            }
        }
        (new SciSparkSpeedCommand(this, this.commandNumber)).start();
    }

    public void set(double speed) {
        set(speed, DEFAULT_MAX_JERK);
    }

    public void moveToGoal() {
        instantSet(this.goalSpeed, this.currentMaxJerk);
    }
    
    public boolean atGoal(){
        return this.goalSpeed == super.get();
    }

    public void updateRobotState(){
        Robot.optionalSet(this.wheelAngleSD, getWheelAngle());
        Robot.optionalSet(this.valueSD,      super.get());
        Robot.optionalSet(this.currentSD,    super.getOutputCurrent());
    }

    public void assignWheelAngleSD(SD wheelAngleSD) {this.wheelAngleSD = Optional.of(wheelAngleSD);}
    public void assignValueSD     (SD valueSD)      {this.valueSD      = Optional.of(valueSD);}
    public void assignCurrentSD   (SD currentSD)    {this.currentSD    = Optional.of(currentSD);}
}