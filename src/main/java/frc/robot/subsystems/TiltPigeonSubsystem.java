package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.*;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Subsystem;

public class TiltPigeonSubsystem extends Subsystem {
    SciPigeon tiltPigeon;
    public TiltPigeonSubsystem() {
        tiltPigeon = new SciPigeon(new SciTalon(PortMap.TILT_PIGEON));
    }

    public void updateRobotState(){
        Robot.set(SD.TiltAngle, tiltPigeon.getAngle());
    }

    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}