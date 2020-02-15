package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import frc.robot.controllers.PID;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.helpers.Geo;
import frc.robot.robotState.StateInfo;
import frc.robot.sciSensorsActuators.*;
import frc.robot.sciSensorsActuators.SciSpark.SciSparkSD;
import frc.robot.logging.LogUpdater;
import frc.robot.logging.Logger.DefaultValue;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveSubsystem extends Subsystem implements LogUpdater {
    // Define tested error values here
    double TANK_ANGLE_P = .075, TANK_ANGLE_D = 0.0, TANK_ANGLE_I = 0;
    double TANK_SPEED_LEFT_P  = .1, TANK_SPEED_LEFT_D  = 0.0, TANK_SPEED_LEFT_I  = 0;
    double TANK_SPEED_RIGHT_P = TANK_SPEED_LEFT_P, TANK_SPEED_RIGHT_D = TANK_SPEED_LEFT_D, TANK_SPEED_RIGHT_I = TANK_SPEED_LEFT_I;
    double GOAL_OMEGA_CONSTANT = 8; // Change this to change angle
    private double MAX_OMEGA_GOAL = 1 * GOAL_OMEGA_CONSTANT;
    public SciSpark l, l1, l2, r, r1, r2;
    public static final double WHEEL_RADIUS = Utils.inchesToMeters(3);

    // deadzones by Alejandro at Chris' request. Graph them with the joystick function to understand the math.
    // https://www.desmos.com/calculator/ch19ahiwol
    private static final double INPUT_DEADZONE = 0.11; // deadzone because the joysticks are bad and they detect input when there is none
    private static final double STRAIGHT_DEADZONE = 0.15;
    private static final double STRAIGHT_EQUAL_INPUT_DEADZONE = 0; // If goal Omega is 0 and our regular input diff magnitude is less than this, the input diff goes to 0
    public static final double GEAR_RATIO = 1 / 19.16;
    public static final boolean RIGHT_INVERTED = true;
    public static final boolean LEFT_INVERTED  = !RIGHT_INVERTED;
    private PID tankAnglePID;
    private PID tankSpeedLeftPID;
    private PID tankSpeedRightPID;
    public boolean assisted = false;
    public boolean reversed = false;
    public double driveMultiplier = 1;

    public PID getTankAnglePID()   {return this.tankAnglePID;}
    public double getMaxOmegaGoal(){return MAX_OMEGA_GOAL;}

    public DriveSubsystem() {

        Robot.set(SD.Angle, 0);

		this.l  = new SciSpark(PortMap.LEFT_FRONT_SPARK,   GEAR_RATIO);
		this.l1 = new SciSpark(PortMap.LEFT_MIDDLE_SPARK,  GEAR_RATIO);
        this.l2 = new SciSpark(PortMap.LEFT_BACK_SPARK,    GEAR_RATIO);
        
		this.r  = new SciSpark(PortMap.RIGHT_FRONT_SPARK,  GEAR_RATIO);
		this.r1 = new SciSpark(PortMap.RIGHT_MIDDLE_SPARK, GEAR_RATIO);
        this.r2 = new SciSpark(PortMap.RIGHT_BACK_SPARK,   GEAR_RATIO);

        this.reversed = false;
        this.setReveresed(false);

        this.l1.follow(this.l);
        this.l2.follow(this.l);

        this.r1.follow(this.r);
        this.r2.follow(this.r);

        this.r.assignSD(SciSparkSD.WheelAngle, SD.RightWheelAngle);
        this.r.assignSD(SciSparkSD.Velocity, SD.RightWheelSpeed);
        this.r.assignSD(SciSparkSD.Accel, SD.RightWheelAccel);
        this.r.assignSD(SciSparkSD.Jerk, SD.RightWheelJerk);
        this.r.assignSD(SciSparkSD.Snap, SD.RightWheelSnap);
        this.r.assignSD(SciSparkSD.Current, SD.RightWheelCurrent);
        this.r.assignSD(SciSparkSD.Value, SD.RightWheelVal);

        this.l.assignSD(SciSparkSD.WheelAngle, SD.LeftWheelAngle);
        this.l.assignSD(SciSparkSD.Velocity, SD.LeftWheelSpeed);
        this.l.assignSD(SciSparkSD.Accel, SD.LeftWheelAccel);
        this.l.assignSD(SciSparkSD.Jerk, SD.LeftWheelJerk);
        this.l.assignSD(SciSparkSD.Snap, SD.LeftWheelSnap);
        this.l.assignSD(SciSparkSD.Current, SD.LeftWheelCurrent);
        this.l.assignSD(SciSparkSD.Value, SD.LeftWheelVal);

        this.r.logAllSDs();
        this.l.logAllSDs();

        this.l.setIdleMode(IdleMode.kCoast);
        this.r.setIdleMode(IdleMode.kCoast);

        // this.l.diminishSnap();
        // this.r.diminishSnap();

        this.tankAnglePID      = new PID(TANK_ANGLE_P,       TANK_ANGLE_I,       TANK_ANGLE_D);
        this.tankSpeedRightPID = new PID(TANK_SPEED_LEFT_P,  TANK_SPEED_LEFT_I,  TANK_SPEED_LEFT_D);
        this.tankSpeedLeftPID  = new PID(TANK_SPEED_RIGHT_P, TANK_SPEED_RIGHT_I, TANK_SPEED_RIGHT_D);

        Robot.logger.logFinalPIDConstants("tank angle PID", this.tankAnglePID);
        Robot.logger.logFinalField       ("input deadzone", INPUT_DEADZONE);

        Robot.addLogUpdater(this);
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

    public void setReveresed(boolean reversed) {
        if(this.reversed != reversed){
            this.reversed = reversed;
            Robot.set(SD.Angle, Robot.get(SD.Angle) + Math.PI);
        }
    
        boolean leftInversion  = LEFT_INVERTED  ^ this.reversed;
        boolean rightInversion = RIGHT_INVERTED ^ this.reversed;
    
        this.l.setInverted (leftInversion);
        this.l1.setInverted(leftInversion);
        this.l2.setInverted(leftInversion);
    
        this.r.setInverted (rightInversion);
        this.r1.setInverted(rightInversion);
        this.r2.setInverted(rightInversion);
    }
        	
	public void setTank(double leftSpeed, double rightSpeed) {
        if (reversed) {
            double temp = leftSpeed;
            leftSpeed = rightSpeed;
            rightSpeed = temp; 
        }
        this.l.set(leftSpeed  * this.driveMultiplier);
        this.r.set(rightSpeed * this.driveMultiplier);

    }

    public void setSpeedTank(double leftGoalSpeed, double rightGoalSpeed){
        this.tankSpeedLeftPID .addMeasurement(leftGoalSpeed  - Robot.get(SD.LeftWheelSpeed));
        this.tankSpeedRightPID.addMeasurement(rightGoalSpeed - Robot.get(SD.RightWheelSpeed));
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
        Robot.logger.addData("input diff", inputDiff, DefaultValue.Empty);
        Robot.logger.addData("error", error, DefaultValue.Empty);
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

    @Override 
    public void periodicLog(){}
}
