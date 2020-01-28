package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciSolenoid;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.PortMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class GearShiftSubsystem extends Subsystem { // example for sciSolenoid

	private final String fileName = "GearShiftSubsystem.java";
    public SciSolenoid gearShiftSolenoid;
    
    private final double UPPER_HIGH_GEAR_THRESHOLD = 1000;
    private final double LOWER_LOW_GEAR_THRESHOLD = 500;
    
    public enum Gear {HIGH, LOW, OFF}

    public GearShiftSubsystem() {
        this.gearShiftSolenoid = new SciSolenoid(PortMap.GEAR_SHIFTER_SOLENOID_PDP, PortMap.GEAR_SHIFTER_SOLENOID, Gear.HIGH, Gear.LOW, Gear.OFF);
        shiftUp();
    }
    
	public void periodicLog(){
        String gear = currentlyInHighGear() ? "high" : "low";
        Robot.logger.addData(this.fileName, "gear", gear, DefaultValue.Previous);
	}
    
    public void autoShift(){
        double speed = Robot.robotPosition.getSpeed();
        if(speed > UPPER_HIGH_GEAR_THRESHOLD){shiftDown();}
        if(speed < LOWER_LOW_GEAR_THRESHOLD) {shiftUp();}
    }

    public boolean currentlyInHighGear(){return this.gearShiftSolenoid.getValue() == Gear.HIGH;}
    public boolean currentlyInLowGear() {return this.gearShiftSolenoid.getValue() == Gear.LOW;}

    public void shiftUp()    {this.gearShiftSolenoid.set(Gear.HIGH);}
    public void shiftDown()  {this.gearShiftSolenoid.set(Gear.LOW);}
    public void toggleGear() {this.gearShiftSolenoid.toggle();}

    @Override
    protected void initDefaultCommand() {
        // LITTERALLY DIE
    }
}