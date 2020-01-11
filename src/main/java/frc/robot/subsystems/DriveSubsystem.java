package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.controllers.PID;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.robotState.StateInfo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.logging.Logger.DefaultValue;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    // Define tested error values here
    double TANK_ANGLE_P = .075, TANK_ANGLE_D = 0.0, TANK_ANGLE_I = 0;
    double GOAL_OMEGA_CONSTANT = 8; // Change this to change angle
    private double MAX_OMEGA_GOAL = 1 * GOAL_OMEGA_CONSTANT;
	private final String FILENAME = "DriveSubsystem.java";
    public static final double WHEEL_RADIUS = Utils.inchesToMeters(3);

    // deadzones by Alejandro at Chris' request. Graph them with the joystick function to understand the math.
    // https://www.desmos.com/calculator/ch19ahiwol
    private static final double INPUT_DEADZONE = 0.11; // deadzone because the joysticks are bad and they detect input when there is none
    private static final double STRAIGHT_DEADZONE = 0.15;
    private static final double STRAIGHT_EQUAL_INPUT_DEADZONE = 0; // If goal Omega is 0 and our regular input diff magnitude is less than this, the input diff goes to 0
    public static final double GEAR_RATIO = 1 / 19.16;

    private PID tankAnglePID;
    public boolean assisted = false;
    public double driveMultiplier = 1;

    public Hashtable<SciSpark, SD> sparkToWheelAngleSD;
    public Hashtable<SciSpark, SD> sparkToValueSD;
    public Hashtable<SciSpark, SD> sparkToVoltageSD;
    public Hashtable<SciSpark, SD> sparkToCurrentSD;

    public PID getTankAnglePID()   {return this.tankAnglePID;}
    public double getMaxOmegaGoal(){return MAX_OMEGA_GOAL;}

    public DriveSubsystem() {
        this.sparkToWheelAngleSD = new Hashtable<>();
        this.sparkToValueSD      = new Hashtable<>();
        this.sparkToVoltageSD    = new Hashtable<>();
        this.sparkToCurrentSD    = new Hashtable<>();

        // Mappings for logging
        setSDMappings(Robot.gearBoxSubsystem.l, SD.LeftWheelAngle, SD.LeftSparkVal,  SD.LeftSparkVoltage,  SD.LeftSparkCurrent);
        setSDMappings(Robot.gearBoxSubsystem.r, SD.RightWheelAngle, SD.RightSparkVal, SD.RightSparkVoltage, SD.RightSparkCurrent);
        
        setSDMappings(Robot.gearBoxSubsystem.l1, SD.L1WheelAngle, SD.L1SparkVal, SD.L1SparkVoltage, SD.L1SparkCurrent);
        setSDMappings(Robot.gearBoxSubsystem.r1, SD.R1WheelAngle, SD.R1SparkVal, SD.R1SparkVoltage, SD.R1SparkCurrent); // Change wheelAngle, access in DriveSubsystem
        setSDMappings(Robot.gearBoxSubsystem.l2, SD.L2WheelAngle, SD.L2SparkVal, SD.L2SparkVoltage, SD.L2SparkCurrent);
        setSDMappings(Robot.gearBoxSubsystem.r2, SD.R2WheelAngle, SD.R2SparkVal, SD.R2SparkVoltage, SD.R2SparkCurrent);

        Robot.addSDToLog(SD.LeftWheelAngle);

        this.tankAnglePID = new PID(TANK_ANGLE_P, TANK_ANGLE_I, TANK_ANGLE_D);
        Robot.logger.logFinalPIDConstants(FILENAME, "tank angle PID", this.tankAnglePID);
        Robot.logger.logFinalField(FILENAME, "input deadzone", INPUT_DEADZONE);
    }

    public void setSDMappings(SciSpark spark, SD wheelAngleSD, SD valueSD, SD volatageSD, SD currentSd){
        this.sparkToWheelAngleSD.put(spark, wheelAngleSD);
        this.sparkToValueSD     .put(spark, valueSD);
        this.sparkToVoltageSD   .put(spark, volatageSD);
        this.sparkToCurrentSD   .put(spark, currentSd);
    }

    public void updateSparkState(SciSpark spark){
        Robot.set(this.sparkToWheelAngleSD.get(spark), spark.getWheelAngle());
        Robot.set(this.sparkToValueSD.get(spark),      spark.get());
        Robot.set(this.sparkToVoltageSD.get(spark),    spark.getBusVoltage());
        Robot.set(this.sparkToCurrentSD.get(spark),    spark.getOutputCurrent());
    }

    public void updateRobotState(){
        for(SciSpark spark : Robot.gearBoxSubsystem.getSparks()){updateSparkState(spark);}
    }

	public void periodicLog(){
    }  

    public double deadzone(double output){
        return Math.abs(output) < INPUT_DEADZONE ? 0 : output;
    }
    
    public double processStick(Joystick stick){
        //return -stick.getY();
        return -deadzone(stick.getY());
    }

    // If something is assiting, we don't want to drive using setSpeed
    public void assistedDriveMode(){this.assisted = true;}
    public void manualDriveMode()  {this.assisted = false;}

    public void setSpeed(Joystick leftStick, Joystick rightStick) {
        if (!this.assisted) {
            double leftInput  = processStick(leftStick);
            double rightInput = processStick(rightStick);
            setSpeedTankAngularControl(leftInput, rightInput);
        }
    }
	
	public void setSpeedRaw(Joystick leftStick, Joystick rightStick){
		setSpeedTank(processStick(leftStick),processStick(rightStick));
    }

    public void defaultDriveMultilpier(){this.driveMultiplier = 1;}
    public void setDriveMultiplier(double driveMultiplier){
        this.driveMultiplier = driveMultiplier;
    }
        	
	public void setSpeedTank(double leftSpeed, double rightSpeed) {
        Robot.gearBoxSubsystem.l.set(leftSpeed  * this.driveMultiplier);
        Robot.gearBoxSubsystem.r.set(rightSpeed * this.driveMultiplier);
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
		setSpeedTank(forward * (1 -  turnMagnitude), forward * (1 + turnMagnitude));
    }
    
    public void setSpeedTankTurningPercentage(double turnMagnitude){
        double forward = (processStick(Robot.oi.leftStick) + processStick(Robot.oi.rightStick)) / 2;
        // double forward = (Robot.oi.leftStick.getY() + Robot.oi.rightStick.getY()) / 2;
        DelayedPrinter.print("Forward: "+ forward);
        DelayedPrinter.print("LeftStick: "+Robot.oi.leftStick.getY() +"\tRightStick: "+ Robot.oi.rightStick.getY());
        setSpeedTankForwardTurningPercentage(forward, turnMagnitude);
	}

    public void setSpeedTankForwardTurningMagnitude(double forward, double turnMagnitude) {
        // Note: this controls dtheta/dt rather than dtheta/dx
        setSpeedTank(forward - turnMagnitude, forward + turnMagnitude);
    }

    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}
