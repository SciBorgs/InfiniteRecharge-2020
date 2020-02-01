package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.helpers.DelayedPrinter;

public class SciSpark extends CANSparkMax {

    private double goalSpeed;
    private double currentMaxJerk;
    public final static double DEFAULT_MAX_JERK = 0.1;
    private int commandNumber;
    private double gearRatio;

    public SciSpark(int port) {
        this(port, 1);
    }

    public SciSpark(int port, double gearRatio) {
        super(port, MotorType.kBrushless);
        this.goalSpeed = 0;
        this.currentMaxJerk = DEFAULT_MAX_JERK;
        this.commandNumber = 0;
        setWheelAngle(0);
        setGearRatio(gearRatio);
        (new SciSparkSpeedCommand(this, this.commandNumber)).start();
    }

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

    public void updateSpeed(double speed) {
        this.set(get() + speed);
    }

    public void singleSet(double speed, double maxJerk) {
        super.set(Utils.limitChange(super.get(), speed, maxJerk));
        DelayedPrinter.print("current: " + super.get(), 5);
    }
    
    public void set(double speed, double maxJerk) {
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        (new SciSparkSpeedCommand(this, this.commandNumber)).start();
    }

    public void set(double speed) {
        set(speed, DEFAULT_MAX_JERK);
    }

    public void moveToGoal(){
        singleSet(this.goalSpeed, this.currentMaxJerk);
    }

    public boolean atGoal(){
        return this.goalSpeed == super.get();
    }

    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;}

}