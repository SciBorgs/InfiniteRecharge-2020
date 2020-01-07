package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.PortMap;
import frc.robot.Utils;

public class IntakeSubsystem extends Subsystem {
    DoubleSolenoid upDownSolenoid;
    TalonSRX intakeMotor;

    private static final DoubleSolenoid.Value OPEN_VALUE   = Value.kForward;
    private static final DoubleSolenoid.Value CLOSED_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);
    public  boolean acceptingCell;
    public static final double INTAKE_SPEED = 0.01; //Temporary

    public IntakeSubsystem() {
        this.upDownSolenoid = new DoubleSolenoid(PortMap.INTAKE_SOLENOID_FORWARD, PortMap.INTAKE_SOLENOID_REVERSE);
        this.intakeMotor = new TalonSRX(PortMap.INTAKE_TALON);
        this.acceptingCell = true;
    }
    public void setIntakeSpeed(final double speed) {
        Robot.driveSubsystem.setMotorSpeed(this.intakeMotor, speed);
    }

    public void closeIntake() {this.upDownSolenoid.set(CLOSED_VALUE);}
    public void openIntake() {this.upDownSolenoid.set(OPEN_VALUE);}

    public void suck() {
        if (this.acceptingCell) {setIntakeSpeed(INTAKE_SPEED);} 
        else {stop();}
    }

    public void stop() {
        setIntakeSpeed(0.0);
    }

    @Override 
    protected void initDefaultCommand() {
        // not needed
    }

}
