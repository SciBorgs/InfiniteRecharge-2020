package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.commands.generalCommands.SciSparkSpeedCommand;
import frc.robot.dataTypes.Deque;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;

public class SciSpark extends CANSparkMax {

    private double goalSpeed;
    private double currentMaxJerk;
    public final static double DEFAULT_MAX_JERK = 0.1;
    private double gearRatio;
    public Optional<SD> wheelAngleSD, valueSD;

    public SciSpark(int port, SD wheelAngleSD) {
        this(port, 1, wheelAngleSD);
    }

    public SciSpark(int port, double gearRatio, SD wheelAngleSD) {
        super(port, MotorType.kBrushless);
        this.goalSpeed = 0;
        this.currentMaxJerk = DEFAULT_MAX_JERK;
        this.wheelAngleSD = wheelAngleSD;
        setWheelAngle(0);
        setGearRatio(gearRatio);
        (new SciSparkSpeedCommand(this)).start();
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

    public void set(double speed, double maxJerk) {
        this.goalSpeed = speed;
        this.currentMaxJerk = maxJerk;
        double limitedInput = Utils.limitChange(super.get(), speed, maxJerk);
        super.set(limitedInput);
        if (limitedInput != super.get()) {
            String warning = "WARNING: Spark " + super.getDeviceId() + " was set to " + limitedInput
                    + " but still has a value of " + super.get();
            System.out.println(warning);
            System.out.println(warning);
            System.out.println(warning);
            System.out.println(warning);
            System.out.println("Debugging info:");
            double positionChange = StateInfo.getFullDifference(this.wheelAngleSD);
            System.out.println("Position Change: " + positionChange);
            System.out.println("Is significant?: " + (Math.abs(positionChange) > Utils.EPSILON));
        }
    }

    public void set(double speed) {set(speed, DEFAULT_MAX_JERK);}
    public void moveToGoal()      {set(this.goalSpeed, this.currentMaxJerk);}

    public boolean atGoal(){
        return this.goalSpeed == super.get();
    }

    public void updateRobotState(){
        Robot.set(this.wheelAngleSD, get());
    }
    public void setValueSD(SD valueSD){
        this.valueSD = valueSD;
    }

}