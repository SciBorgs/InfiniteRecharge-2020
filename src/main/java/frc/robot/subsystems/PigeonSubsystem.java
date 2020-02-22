package frc.robot.subsystems;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.sciSensorsActuators.SciPigeon.SciPigeonSD;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class PigeonSubsystem implements Subsystem {
  // for the main pigeon on the robot
    public  SciPigeon pigeon;
    private TalonSRX pigeonTalon;
    
    public PigeonSubsystem () {
        this.pigeonTalon = new TalonSRX(PortMap.PIGEON_TALON);
        this.pigeon      = new SciPigeon(pigeonTalon);
        this.pigeon.assignSD(SciPigeonSD.Angle, SD.PigeonAngle);
    }

    public SciPigeon getPigeon() {return this.pigeon;}
}