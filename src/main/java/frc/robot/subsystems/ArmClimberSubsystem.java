package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.controllers.PID;
import frc.robot.sciSensorsActuators.SciTalon;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.sciSensorsActuators.SciSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ArmClimberSubsystem extends Subsystem {
    private SciTalon armTalon;
    private SciTalon stringTalon1, stringTalon2;
    private SciSolenoid armSolenoid;
    private PID talonPID;

    public enum Position {OPEN, CLOSED, OFF} // gets rid of fields

    private final double TALON_GEAR_RATIO = 1;
    private final double SPARK_GEAR_RATIO = 1;
    private final double ARM_LENGTH = 1;
    private final double STRING_PULL_SPEED = 0.2;
    private final double STRING_PUSH_SPEED = -0.2;
    private final double RAISE_SPEED = 0.35;
    private final double LOWER_SPEED = -0.55;

    public ArmClimberSubsystem() {
        this.armTalon     = new SciTalon(PortMap.ARM_TALON,      TALON_GEAR_RATIO);
        this.stringTalon1 = new SciTalon(PortMap.STRING_SPARK_1, SPARK_GEAR_RATIO);
        this.stringTalon2 = new SciTalon(PortMap.STRING_SPARK_2, SPARK_GEAR_RATIO);

        this.armSolenoid = new SciSolenoid(PortMap.ARM_SOLENOID_PDP, PortMap.ARM_SOLENOID, Position.OPEN, Position.CLOSED, Position.OFF);
        this.talonPID = new PID(1,0,0);

        this.stringTalon2.follow(stringTalon1);

        closeSolenoidArm();
    }

    public void setHeight(double height) {
        double targetAngle = Math.asin(height / (2 * ARM_LENGTH));

        this.talonPID.addMeasurement(targetAngle - this.armTalon.getWheelAngle());
        setArmSpeed(this.talonPID.getOutput());
    }

    public void setArmSpeed(double speed) {
        this.armTalon.set(speed, 2);
        System.out.println("!!!!!!!!!!!!!!!!!!Setting to speed: " + speed);
    }

    public void raiseArm() {setArmSpeed(RAISE_SPEED);}
    public void lowerArm() {setArmSpeed(LOWER_SPEED);}
    public void stopArm()  {setArmSpeed(0);}
    public void armChange(double change){
        System.out.println("current arm output percent: " + this.armTalon.getMotorOutputPercent());
        setArmSpeed(this.armTalon.getMotorOutputPercent() + change);
    }

    public void setStringSpeed(double speed){
        this.stringTalon1.set(speed, 2);
    }

    public void pullString() {setStringSpeed(STRING_PULL_SPEED);}
    public void pushString() {setStringSpeed(STRING_PUSH_SPEED);}
    public void stopString() {setStringSpeed(0);}
    public void stringChange(double change){
        System.out.println("current string output percent: " + this.stringTalon1.getMotorOutputPercent());
        setStringSpeed(this.stringTalon1.getMotorOutputPercent() + change);
    }

    public void openSolenoidArm()  {this.armSolenoid.set(Position.OPEN);} // !!
    public void closeSolenoidArm() {this.armSolenoid.set(Position.CLOSED);}
    public void toggleSolenoidArm() {this.armSolenoid.toggle());}

    @Override
    protected void initDefaultCommand() {} // but why tho
}