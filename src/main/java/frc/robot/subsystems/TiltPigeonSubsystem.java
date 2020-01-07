package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.*;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Subsystem;

public class TiltPigeonSubsystem extends Subsystem {
    private SciPigeon tiltPigeon;
    private final String FILENAME = "TiltPigeonSubsystem.java";

    public TiltPigeonSubsystem() {
        this.tiltPigeon = new SciPigeon(new SciTalon(PortMap.TILT_PIGEON));
        Robot.addSDToLog(SD.TiltAngle);
    }

    public void setAngle(double angle) {
        this.tiltPigeon.setAngle(angle);
    }

    public void updateRobotState(){
        Robot.set(SD.TiltAngle, this.tiltPigeon.getAngle());
    }

    @Override
    protected void initDefaultCommand() {
		//IGNORE THIS METHOD
    }
}