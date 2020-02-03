package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import frc.robot.controllers.PID;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.StateInfo;
import frc.robot.sciSensorsActuators.*;
import frc.robot.logging.Logger.DefaultValue;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    // Define tested error values here
    double TANK_ANGLE_P = .075, TANK_ANGLE_D = 0.0, TANK_ANGLE_I = 0;
    double TANK_SPEED_LEFT_P  = .1, TANK_SPEED_LEFT_D  = 0.0, TANK_SPEED_LEFT_I  = 0;
    double TANK_SPEED_RIGHT_P = TANK_SPEED_LEFT_P, TANK_SPEED_RIGHT_D = TANK_SPEED_LEFT_D, TANK_SPEED_RIGHT_I = TANK_SPEED_LEFT_I;
    double GOAL_OMEGA_CONSTANT = 8; // Change this to change angle
    private double MAX_OMEGA_GOAL = 1 * GOAL_OMEGA_CONSTANT;
    public SciSpark l, l1, l2, r, r1, r2;
    private final String FILENAME = "DriveSubsystem.java";
    public static final double WHEEL_RADIUS = Utils.inchesToMeters(3);

    // deadzones by Alejandro at Chris' request. Graph them with the joystick function to understand the math.
    // https://www.desmos.com/calculator/ch19ahiwol
    private static final double INPUT_DEADZONE = 0.11; // deadzone because the joysticks are bad and they detect input when there is none
    private static final double STRAIGHT_DEADZONE = 0.15;
    private static final double STRAIGHT_EQUAL_INPUT_DEADZONE = 0; // If goal Omega is 0 and our regular input diff magnitude is less than this, the input diff goes to 0
    public static final double GEAR_RATIO = 1 / 19.16;
    private PID tankAnglePID;
    private PID tankSpeedLeftPID;
    private PID tankSpeedRightPID;
    public boolean assisted = false;
    public double driveMultiplier = 1;

    public PID getTankAnglePID()   {return this.tankAnglePID;}
    public double getMaxOmegaGoal(){return MAX_OMEGA_GOAL;}

    public DriveSubsystem() {

		this.l  = new SciSpark(PortMap.LEFT_FRONT_SPARK,   GEAR_RATIO);
		this.l1 = new SciSpark(PortMap.LEFT_MIDDLE_SPARK,  GEAR_RATIO);
        this.l2 = new SciSpark(PortMap.LEFT_BACK_SPARK,    GEAR_RATIO);
        
		this.r  = new SciSpark(PortMap.RIGHT_FRONT_SPARK,  GEAR_RATIO);
		this.r1 = new SciSpark(PortMap.RIGHT_MIDDLE_SPARK, GEAR_RATIO);
        this.r2 = new SciSpark(PortMap.RIGHT_BACK_SPARK,   GEAR_RATIO);

        this.r .setInverted(true);
        this.r1.setInverted(true);
        this.r2.setInverted(true);

        this.l1.follow(this.l);
        this.l2.follow(this.l);

        this.r1.follow(this.r);
        this.r2.follow(this.r);

        this.r.assignAll(SD.RightWheelAngle, SD.RightSparkVal, SD.RightCurrentVal);
        this.l.assignAll(SD.LeftWheelAngle,  SD.LeftSparkVal,  SD.LeftCurrentVal);

        Robot.addSDToLog(SD.LeftWheelAngle);
        Robot.addSDToLog(SD.RightWheelAngle);
        Robot.addSDToLog(SD.LeftSparkVal);
        Robot.addSDToLog(SD.RightSparkVal);

        this.tankAnglePID      = new PID(TANK_ANGLE_P,       TANK_ANGLE_I,       TANK_ANGLE_D);
        this.tankSpeedRightPID = new PID(TANK_SPEED_LEFT_P,  TANK_SPEED_LEFT_I,  TANK_SPEED_LEFT_D);
        this.tankSpeedLeftPID  = new PID(TANK_SPEED_RIGHT_P, TANK_SPEED_RIGHT_I, TANK_SPEED_RIGHT_D);

        Robot.logger.logFinalPIDConstants(FILENAME, "tank angle PID", this.tankAnglePID);
        Robot.logger.logFinalField       (FILENAME, "input deadzone", INPUT_DEADZONE);
    }

	public void periodicLog(){
    }

    // If something is assiting, we don't want to drive using setSpeed
    public void assistedDriveMode(){this.assisted = true;}
    public void manualDriveMode()  {this.assisted = false;}

    public void setSpeed(SciJoystick leftStick, SciJoystick rightStick) {
        if (!this.assisted) {
            double leftInput  = leftStick.getProcessedY();
            double rightInput = rightStick.getProcessedY();
            setSpeedTankAngularControl(leftInput, rightInput);
        }
    }
	
	public void setSpeedRaw(SciJoystick leftStick, SciJoystick rightStick){
		setTank(leftStick.getProcessedY(), rightStick.getProcessedY());
    }

    public void defaultDriveMultilpier(){this.driveMultiplier = 1;}
    public void setDriveMultiplier(double driveMultiplier){
        this.driveMultiplier = driveMultiplier;
    }
        	
	public void setTank(double leftSpeed, double rightSpeed) {
        this.l.set(leftSpeed  * this.driveMultiplier);
        this.r.set(rightSpeed * this.driveMultiplier);
    }

    public void setSpeedTank(double leftGoalSpeed, double rightGoalSpeed){
        double currentLeft  = StateInfo.getWheelSpeed(this.l);
        double currentRight = StateInfo.getWheelSpeed(this.r);
        this.tankSpeedLeftPID.addMeasurement(leftGoalSpeed - currentLeft);
        this.tankSpeedRightPID.addMeasurement(rightGoalSpeed - currentRight);
        setTank(tankSpeedLeftPID.getOutput(), tankSpeedRightPID.getOutput());
    }
	
	public void setSpeedTankAngularControl(double leftSpeed, double rightSpeed) {
		double averageOutput = (leftSpeed + rightSpeed) / 2;
        double goalOmega = GOAL_OMEGA_CONSTANT * (rightSpeed - leftSpeed);
        // Makes it so if the two joysticks are close enough, it will try to go straight
        if (Math.abs(goalOmega) < STRAIGHT_DEADZONE){
            goalOmega = 0;
        } else {
            goalOmega -= Utils.signOf(goalOmega) * STRAIGHT_DEADZONE;
        }
        goalOmega = Utils.limitOutput(goalOmega, MAX_OMEGA_GOAL);
        double error = StateInfo.getAngularVelocity() - goalOmega;
        tankAnglePID.addMeasurement(error);
        double inputDiff = tankAnglePID.getOutput();
        // If you are going almost straight and goalOmega is 0, it will simply give the same input to both wheels
        // We should test if this is beneficial
        if (goalOmega == 0 && (Math.abs(inputDiff) < STRAIGHT_EQUAL_INPUT_DEADZONE)){
            inputDiff = 0;
        }
        Robot.logger.addData(FILENAME, "input diff", inputDiff, DefaultValue.Empty);
        Robot.logger.addData(FILENAME, "error", error, DefaultValue.Empty);
		setSpeedTankForwardTurningMagnitude(averageOutput, inputDiff);
	}
	
	public void setSpeedTankForwardTurningPercentage(double forward, double turnMagnitude) {
        // Note: this controls dtheta/dx rather than dtheta/dt
		setSpeedTank((forward / TANK_SPEED_LEFT_P) * (1 -  turnMagnitude), (forward / TANK_SPEED_RIGHT_P) * (1 + turnMagnitude));
    }
    
    public void setSpeedTankTurningPercentage(double turnMagnitude){
        double forward = (Robot.oi.leftStick.getProcessedY() + Robot.oi.rightStick.getProcessedY()) / 2;
        // double forward = (Robot.oi.leftStick.getY() + Robot.oi.rightStick.getY()) / 2;
        setSpeedTankForwardTurningPercentage(forward, turnMagnitude);
	}

    public void setSpeedTankForwardTurningMagnitude(double forward, double turnMagnitude) {
        // Note: this controls dtheta/dt rather than dtheta/dx
        setTank(forward - turnMagnitude, forward + turnMagnitude);
    }

    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}
