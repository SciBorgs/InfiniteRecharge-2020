package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Utils;

public class DualOutputGearboxSubsystem extends Subsystem {
    public final DoubleSolenoid.Value OPEN_VALUE = Value.kForward;
    public final DoubleSolenoid.Value CLOSE_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);

    private DoubleSolenoid GEAR_SHIFT_SOLENOID;
    private DoubleSolenoid OUTPUT_SHIFT_SOLENOID;

    enum Gearbox {
        Climber(2),
        Drive(0);
        
        private double gearRatio;

        Gearbox(double gearRatio) {
            this.gearRatio = gearRatio;
        }

        public double getGearRatio() {
            return gearRatio;
        }
    }

    public DualOutputGearboxSubsystem() {
        GEAR_SHIFT_SOLENOID   = Utils.newDoubleSolenoid(PortMap.GEAR_RATIO_SOLENOID_PDP, PortMap.GEAR_RATIO_SOLENOID);
        OUTPUT_SHIFT_SOLENOID = Utils.newDoubleSolenoid(PortMap.OUTPUT_SOLENOID_PDP, PortMap.OUTPUT_SOLENOID);
    }
    
    public void initDefaultCommand() {}

    public void shiftGearsPush()     { GEAR_SHIFT_SOLENOID.set(OPEN_VALUE); }

    public void shiftGearRetract()   { GEAR_SHIFT_SOLENOID.set(CLOSE_VALUE); }

    public void shiftOutputPush()    { OUTPUT_SHIFT_SOLENOID.set(OPEN_VALUE); }

    public void shiftOutputRetract() { OUTPUT_SHIFT_SOLENOID.set(CLOSE_VALUE); }

    
}