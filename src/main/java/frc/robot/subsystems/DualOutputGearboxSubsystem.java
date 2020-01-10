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
    
    // Need actual gear ratios.
    private final double SHIFT_UP_GEAR_RATIO = 0;
    private final double SHIFT_DOWN_GEAR_RATIO = 0;

    private DoubleSolenoid gearShiftSolenoid;
    private DoubleSolenoid outputShiftSolenoid;

    public SciSpark l, l1, l2, r, r1, r2;

    public DualOutputGearboxSubsystem() {
        gearShiftSolenoid   = Utils.newDoubleSolenoid(PortMap.GEAR_RATIO_SOLENOID_PDP, PortMap.GEAR_RATIO_SOLENOID);
        outputShiftSolenoid = Utils.newDoubleSolenoid(PortMap.OUTPUT_SOLENOID_PDP, PortMap.OUTPUT_SOLENOID);
        
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
        gearShiftSolenoid.set(SHIFT_GEAR_UP_VALUE);
        setGearRatio(SHIFT_UP_GEAR_RATIO);
    } 
    public void shiftGearDown() { 
        gearShiftSolenoid.set(SHIFT_GEAR_DOWN_VALUE); 
        setGearRatio(SHIFT_DOWN_GEAR_RATIO);
    }
    
    public void shiftClimberOutput() { 
        outputShiftSolenoid.set(SHIFT_CLIMBER_OUTPUT_VALUE); 
    }

    public void shiftDriveOutput() { 
        outputShiftSolenoid.set(SHIFT_DRIVE_OUTPUT_VALUE); 
    }
    
    public SciSpark[] getSparks() {
        return new SciSpark[]{this.l, this.l1, this.l2, this.r, this.r1, this.r2};
    }

    private void setGearRatio(double gearRatio) {
        for (SciSpark spark : getSparks()){spark.setGearRatio(gearRatio);}
    }
    
    public void initDefaultCommand() {
        // Please ignore.
    }
}