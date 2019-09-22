package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Utils;
import frc.robot.RobotState.RS;
import frc.robot.helpers.StateInfo;
import frc.robot.logging.Logger.DefaultValue;
import frc.robot.PortMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class GearShiftSubsystem extends Subsystem {

	private final String FILENAME = "GearShiftSubsystem.java";
    public DoubleSolenoid gearShiftSolenoid;
    

    public static final double LOW_GEAR_RATIO = 1 / 19.16; // 1 rotations of the wheel is 9.08 rotations of the encoder
    public static final double HIGH_GEAR_RATIO = 1 / 9.07;

    private final double UPPER_HIGH_GEAR_THRESHOLD = 1000;
    private final double LOWER_LOW_GEAR_THRESHOLD = 500;
	public static final DoubleSolenoid.Value HIGH_GEAR_VALUE = Value.kForward;
	public static final DoubleSolenoid.Value LOW_GEAR_VALUE = Utils.oppositeDoubleSolenoidValue(HIGH_GEAR_VALUE);
    public static final RS GEAR_SHIFT_RS = RS.GearShiftSolenoid;

    public GearShiftSubsystem() {
        this.gearShiftSolenoid = Utils.newDoubleSolenoid(PortMap.GEAR_SHIFTER_SOLENOID_PDP, PortMap.GEAR_SHIFTER_SOLENOID);
        shiftUp();
    }
    
	public void periodicLog(){
        String gear = currentlyInHighGear() ? "high" : "low";
        Robot.logger.addData(FILENAME, "gear", gear, DefaultValue.Previous);
    }
    public void updateRobotState(){
        Robot.getState().set(GEAR_SHIFT_RS, RobotState.solenoidValueToInt(this.gearShiftSolenoid.get()));
    }
    
    public void autoShift(){
        double speed = StateInfo.getSpeed();
        if(speed > UPPER_HIGH_GEAR_THRESHOLD){shiftDown();}
        if(speed < LOWER_LOW_GEAR_THRESHOLD) {shiftUp();}
    }

    public boolean currentlyInHighGear(){return Robot.getSolenoidValue(GEAR_SHIFT_RS) == HIGH_GEAR_VALUE;}
    public boolean currentlyInLowGear() {return Robot.getSolenoidValue(GEAR_SHIFT_RS) == LOW_GEAR_VALUE;}

    public void shiftUp()  {this.gearShiftSolenoid.set(HIGH_GEAR_VALUE);}
    public void shiftDown(){this.gearShiftSolenoid.set(LOW_GEAR_VALUE);}

    public void toggleGear() {Utils.toggleDoubleSolenoid(this.gearShiftSolenoid);}

    public double getCurrentGearRatio() {
        return currentlyInHighGear() ? HIGH_GEAR_RATIO : LOW_GEAR_RATIO;
    }

    @Override
    protected void initDefaultCommand() {
        // LITTERALLY DIE
    }
}
