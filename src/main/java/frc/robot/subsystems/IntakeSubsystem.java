package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.robotState.RobotState.SD;
import frc.robot.PortMap;
import frc.robot.Utils;

public class IntakeSubsystem extends Subsystem {
    DoubleSolenoid upDownSolenoid;
    TalonSRX intakeMotor;

    private final DoubleSolenoid.Value OPEN_VALUE   = Value.kForward;
    private final DoubleSolenoid.Value CLOSED_VALUE = Utils.oppositeDoubleSolenoidValue(OPEN_VALUE);
    public  boolean AcceptingCell;
    public final double INTAKE_SPEED = 0.01; //Temporary

    public IntakeSubsystem() {
        this.upDownSolenoid = new DoubleSolenoid(PortMap.INTAKE_SOLENOID_FORWARD, PortMap.INTAKE_SOLENOID_REVERSE);
        this.intakeMotor = new TalonSRX(PortMap.INTAKE_TALON);
        AcceptingCell = true;
    }
    void SetIntakeSpeed(final double speed) {
        Robot.driveSubsystem.setMotorSpeed(this.intakeMotor, speed);
    }

    void closeIntake()        {this.upDownSolenoid.set(CLOSED_VALUE);}
    void openIntake()         {this.upDownSolenoid.set(OPEN_VALUE);}
    int  getEnergyBallCount() {return (int)Robot.get(SD.EnergyBallCount);}
    boolean getAcceptingCell(){return getEnergyBallCount() < 5;}
    void setAcceptingCell()   {this.AcceptingCell = getAcceptingCell();}
    void addBallCount()       {Robot.set(SD.EnergyBallCount, getEnergyBallCount()+1);}

    void takeInCell() {
        if (this.AcceptingCell) {
            closeIntake();
            SetIntakeSpeed(this.INTAKE_SPEED);
            addBallCount();
        } else {
            closeIntake();
            SetIntakeSpeed(0.0);
        }
    }

    @Override 
    protected void initDefaultCommand() {

    }

}
