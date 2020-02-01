package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.PortMap;
import frc.robot.Utils;
import frc.robot.sciSensorsActuators.SciTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class IntakeSubsystem extends Subsystem {
    DoubleSolenoid upDownSolenoid;
    public SciTalon intakeMotor;

    private static final DoubleSolenoid.Value OPEN_VALUE   = Value.kForward;
    private static final DoubleSolenoid.Value CLOSED_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);
    public  boolean acceptingCell;
    public static final double INTAKE_SPEED = 1; //Temporary

    public IntakeSubsystem() {
        this.upDownSolenoid = new DoubleSolenoid(PortMap.INTAKE_SOLENOID_FORWARD, PortMap.INTAKE_SOLENOID_REVERSE);
        this.intakeMotor = new SciTalon(PortMap.INTAKE_TALON);
        this.acceptingCell = true;
    }
    public void setIntakeSpeed(double speed) {
        this.intakeMotor.set(speed);
    }

    public void forwardIntake() {this.upDownSolenoid.set(CLOSED_VALUE);}
    public void reverseIntake() {this.upDownSolenoid.set(OPEN_VALUE);}

    public void suck() {setIntakeSpeed(INTAKE_SPEED);}
    public void stop() {setIntakeSpeed(0);}

    @Override 
    protected void initDefaultCommand() {
        // Useless
    }

}