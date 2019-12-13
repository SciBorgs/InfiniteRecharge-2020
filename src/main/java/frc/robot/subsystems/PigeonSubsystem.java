package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.RobotState.SD;
import frc.robot.helpers.Pigeon;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class PigeonSubsystem extends Subsystem {
    public  Pigeon pigeon;
    private TalonSRX pigeonTalon;
    
    public PigeonSubsystem () {
        this.pigeonTalon = new TalonSRX(PortMap.PIGEON_TALON);
        this.pigeon      = new Pigeon(pigeonTalon);
    }

    public Pigeon getPigeon() {return this.pigeon;}
    public void   setAngle()  {Robot.set(SD.PigeonAngle, getPigeon().getAngle());}
    public void   updateRobotState () { setAngle(); }

    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}