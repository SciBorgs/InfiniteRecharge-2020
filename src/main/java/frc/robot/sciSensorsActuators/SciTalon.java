package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciTalonSpeedCommand;
import frc.robot.helpers.DelayedPrinter;

public class SciTalon extends TalonSRX {

    public static final double DEFAULT_MAX_JERK = 0.1;
    public static final double TICKS_PER_REV = 4096;
    public static final double TICKS_PER_RADIAN = TICKS_PER_REV / (2 * Math.PI);
    private int commandNumber;
    private double wheelAngleOffset = 0;
    public double goalSpeed;
    public double currentMaxJerk;
    public double gearRatio;

    public SciTalon(int port) {
        this(port, 1);
    }

    public SciTalon(int port, double gearRatio) {
        super(port);
        this.goalSpeed = 0;
        this.currentMaxJerk = 0;
        setWheelAngle(0);
        setGearRatio(gearRatio);
        this.commandNumber = 0;
    }

    public boolean isCurrentCommandNumber(int n){return n == this.commandNumber;}

    public double getGearRatio() {
        return this.gearRatio;
    }

    public void setGearRatio(double gearRatio) {
        double currentAngle = getWheelAngle();
        this.gearRatio = gearRatio;
        // W/o this when we changed gear ratios, it would weirdly change the angle
        // For instance, if you have a gear ratio of 1/5 and your getPosition() is 20 radians
        // and then you change the gear ratio to 1/10, your getPosition() will go to 10 radians
        setWheelAngle(currentAngle);

    }

    private double ticksToAngle(int ticks) {
        return ticks / TICKS_PER_RADIAN * this.gearRatio;
    }

    private int angleToTicks(double angle) {
        return (int) (angle * TICKS_PER_RADIAN / this.gearRatio);
    }

    public double getWheelAngle() {
        return ticksToAngle(super.getSensorCollection().getQuadraturePosition());
    }

    public void setWheelAngle(double angle) {
        super.getSensorCollection().setQuadraturePosition(angleToTicks(angle), 0);
    }

    public void instantSet(double speed, double maxJerk) {
        super.set(ControlMode.PercentOutput, Utils.limitChange(super.getMotorOutputPercent(), speed, maxJerk));
    }

    public void set(double speed, double maxJerk){
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        this.commandNumber++;
        (new SciTalonSpeedCommand(this, this.commandNumber)).start();
    }

    public void set(double speed) {
        set(speed, DEFAULT_MAX_JERK);
    }
    public void moveToGoal(){
        instantSet(this.goalSpeed, this.currentMaxJerk);
    }

    public boolean atGoal(){
        return this.goalSpeed == super.getMotorOutputPercent();
    }
}