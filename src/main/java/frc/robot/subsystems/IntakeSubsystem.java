package frc.robot.subsystems;

import frc.robot.Utils;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciSolenoid;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeSubsystem extends Subsystem {

    public TalonSRX intakeTalon;
    public SciSolenoid secureHatchSolenoid, armSolenoid, popHatchSolenoid;
    public enum Hatch {RELEASE, SECURE, OFF}
    public enum Arm   {OPEN, CLOSED, OFF}
    public enum Pop   {EXTEND, RETRACT, OFF}
	private Timer timer;
	private final String filename = "IntakeSubsystem.java";
	public final static double SUCK_SPEED = -1;
	public final static double SPIT_SPEED = SUCK_SPEED * -1;
	public final static double PICKUP_HATCH_SPEED = -0.3;
	public final static double SUCK_IF_OUT_PERIOD = 1; // The amount of time that the intake should suck if the ball stops pressing the button in seconds
	public final static double SECURE_CARGO_SPEED = SUCK_SPEED / 2;
	private boolean holdingCargo;

    public IntakeSubsystem() {
		this.timer = new Timer();
		this.timer.start();
		this.intakeTalon = new TalonSRX(PortMap.INTAKE_TALON);
		this.intakeTalon.setNeutralMode(NeutralMode.Brake);
		this.intakeTalon.configContinuousCurrentLimit(10);
		this.intakeTalon.configPeakCurrentLimit(10);
		this.intakeTalon.enableCurrentLimit(true);
		this.holdingCargo = false;
		this.secureHatchSolenoid = new SciSolenoid(PortMap.SECURE_HATCH_SOLENOID_PDP, PortMap.SECURE_HATCH_SOLENOID, Hatch.RELEASE, Hatch.SECURE, Hatch.OFF);
		this.armSolenoid         = new SciSolenoid(PortMap.ARM_SOLENOID_PDP, PortMap.ARM_SOLENOID, Arm.OPEN, Arm.CLOSED, Arm.OFF);
		this.popHatchSolenoid    = new SciSolenoid(PortMap.POP_HATCH_SOLENOID_PDP, PortMap.POP_HATCH_SOLENOID, Pop.RETRACT, Pop.EXTEND, Pop.OFF);
	}
    
	public void periodicLog(){
	}

	public TalonSRX[] getTalons() {
    	return new TalonSRX[]{this.intakeTalon};
	}

	public void releaseHatch() {this.secureHatchSolenoid.set(Hatch.RELEASE);}
	public void secureHatch()  {this.secureHatchSolenoid.set(Hatch.SECURE);}

	public void openArm()   {this.armSolenoid.set(Arm.OPEN);}
	public void closeArm()  {this.armSolenoid.set(Arm.CLOSED);}
	public void toggleArm() {this.armSolenoid.toggle();}

	public void extendPopHatchPistons() {this.popHatchSolenoid.set(Pop.EXTEND);}
	public void retractPopHatchPistons(){this.popHatchSolenoid.set(Pop.RETRACT);}

	public boolean holdingCargo() { // assumes when the drvier tries to intake cargo, the robot actually has it
		return this.holdingCargo;
	}

	public void setIntakeSpeed(double speed){
		Robot.driveSubsystem.setMotorSpeed(this.intakeTalon, -speed, 2);
	}

    public void suck() {
		this.holdingCargo = true; // We assume that sucknig means we have the cargo. W/o limit switches it is the best we can do
		setIntakeSpeed(SUCK_SPEED);
    }

    public void spit() {
		this.holdingCargo = false;
        setIntakeSpeed(SPIT_SPEED);
	}

	public void secureCargo() {
		this.holdingCargo = true;
		setIntakeSpeed(SECURE_CARGO_SPEED);
	}

    @Override
    protected void initDefaultCommand() {
        // LITTERALLY DIE
	}
}