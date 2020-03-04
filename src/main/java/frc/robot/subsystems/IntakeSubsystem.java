package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.sciSensorsActuators.SciSolenoid;
import frc.robot.sciSensorsActuators.SciSpark;


public class IntakeSubsystem extends Subsystem {
    public SciSolenoid<IntakeValue> upDownSolenoid;
    public SciSpark intakeSpark;
    public static enum IntakeValue{Down, Up, Off}

    public static final double INTAKE_SPEED = 1; //Temporary

    public IntakeSubsystem() {
        this.upDownSolenoid = new SciSolenoid<IntakeValue>(PortMap.INTAKE_SOLENOID_PORTS, IntakeValue.Up, IntakeValue.Down, IntakeValue.Off);
        this.intakeSpark = new SciSpark(PortMap.INTAKE_SPARK);
    }
    public void setIntakeSpeed(double speed) {
        this.intakeSpark.set(speed);
    }

    public void suck() {setIntakeSpeed(INTAKE_SPEED);}
    public void stop() {setIntakeSpeed(0);}

    @Override 
    protected void initDefaultCommand() {
        // Useless
    }

}