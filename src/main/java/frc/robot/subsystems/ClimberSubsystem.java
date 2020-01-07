package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.controllers.PID;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    private SciSpark liftRight;
    private SciSpark liftLeft;
    private SciSpark shiftMotor;
    private DoubleSolenoid attachSolenoid;

    private final double CASCADE_GEAR_RATIO = 1; // CHANGE
    private final double CASCADE_WHEEL_RADIUS = 3; //CHANGE
    private final DoubleSolenoid.Value OPEN_VALUE = kForward;
    private final DoubleSolenoid.Value CLOSED_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);

    public ClimberSubsystem() {
        this.liftRight  = new SciSpark(PortMap.LIFT_RIGHT_TALON, CASCADE_GEAR_RATIO);
        this.liftLeft   = new SciSpark(PortMap.LIFT_LEFT_TALON,  CASCADE_GEAR_RATIO);
        this.shiftMotor = new SciSpark(PortMap.SHIFTING_TALON,   CASCADE_GEAR_RATIO);
        this.attachSolenoid = new Utils.newDoubleSolenoid(PortMap.CLIMBER_SOLENOID);

        Robot.addSDToLog(SD.ClimberHeight);
    }

    public void open()  {this.attachSolenoid.set(OPEN_VALUE);}
    public void close() {this.attachSolenoid.set(CLOSED_VALUE);}

    public void setShiftMotorSpeed(double speed) {this.shiftMotor.set(speed);}

    public void setLiftSpeed(double speed){ 
        this.liftRight.set(speed);
        this.liftLeft.set(speed);
    }

    public double getCascadeHeight(){
        return Robot.get(SD.ClimberSparkAngle) * CASCADE_WHEEL_RADIUS;
    }

    public void updateRobotState(){
        Robot.set(SD.ClimberSparkAngle,  this.liftLeft.getWheelAngle());
        Robot.set(SD.ShiftSparkAngle,    this.shiftMotor.getWheelAngle());
        Robot.set(SD.ClimberHeight, getCascadeHeight());
    }
    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}