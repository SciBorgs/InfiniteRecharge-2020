package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.sciSensorsActuators.SciSpark;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DualOutputGearboxSubsystem extends Subsystem { // Set ratio, motor, change shaft
    private final String FILENAME = "DualOutputGearboxSubsystem.java";
    
    public final DoubleSolenoid.Value SHIFT_GEAR_UP_VALUE = Value.kForward; // Place values plz
    public final DoubleSolenoid.Value SHIFT_GEAR_DOWN_VALUE = Utils.oppositeDoubleSolenoidValue(SHIFT_GEAR_UP_VALUE);

    public final DoubleSolenoid.Value SHIFT_CLIMBER_OUTPUT_VALUE = Value.kForward; // Place values plz
    public final DoubleSolenoid.Value SHIFT_DRIVE_OUTPUT_VALUE = Utils.oppositeDoubleSolenoidValue(SHIFT_CLIMBER_OUTPUT_VALUE);

    private DoubleSolenoid GEAR_SHIFT_SOLENOID;
    private DoubleSolenoid OUTPUT_SHIFT_SOLENOID;

    private double driveGearRatio = 0;
    private double climbGearRatio = 0;

    private double shiftUpGearRatio = 0;
    private double shiftDownGearRatio = 0;

    public SciSpark l, l1, l2, r, r1, r2;

    public double shiftGearRatio;
    public double outputGearRatio;

    public DualOutputGearboxSubsystem() {
        GEAR_SHIFT_SOLENOID   = Utils.newDoubleSolenoid(PortMap.GEAR_RATIO_SOLENOID_PDP, PortMap.GEAR_RATIO_SOLENOID);
        OUTPUT_SHIFT_SOLENOID = Utils.newDoubleSolenoid(PortMap.OUTPUT_SOLENOID_PDP, PortMap.OUTPUT_SOLENOID);
        
        this.l  = new SciSpark(PortMap.LEFT_FRONT_SPARK);
		this.l1 = new SciSpark(PortMap.LEFT_MIDDLE_SPARK);
        this.l2 = new SciSpark(PortMap.LEFT_BACK_SPARK);
        
		this.r  = new SciSpark(PortMap.RIGHT_FRONT_SPARK);
		this.r1 = new SciSpark(PortMap.RIGHT_MIDDLE_SPARK);
        this.r2 = new SciSpark(PortMap.RIGHT_BACK_SPARK);

        this.l .setInverted(true);
        this.l1.setInverted(true);
        this.l2.setInverted(true);

        this.l1.follow(this.l);
        this.l2.follow(this.l);

        this.r1.follow(this.r);
        this.r2.follow(this.r);
    }
    
    public void shiftGearsUp() { 
        GEAR_SHIFT_SOLENOID.set(SHIFT_GEAR_UP_VALUE);
        shiftGearRatio = shiftUpGearRatio;
    } 
    public void shiftGearDown() { 
        GEAR_SHIFT_SOLENOID.set(SHIFT_GEAR_DOWN_VALUE); 
        shiftGearRatio = shiftDownGearRatio;
    }
    
    public void shiftClimberOutput() { 
        OUTPUT_SHIFT_SOLENOID.set(SHIFT_CLIMBER_OUTPUT_VALUE); 
        outputGearRatio = climbGearRatio;
    }

    public void shiftDriveOutput() { 
        OUTPUT_SHIFT_SOLENOID.set(SHIFT_DRIVE_OUTPUT_VALUE); 
        outputGearRatio = driveGearRatio;
    }
    
    public SciSpark[] getSparks() {
        return new SciSpark[]{this.l, this.l1, this.l2, this.r, this.r1, this.r2};
    }
    
    public void initDefaultCommand() {
        // Please ignore.
    }
}