package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.PortMap;

public class HopperSubsystem {
    
    public SciSpark upSpark, inSpark;
    private static final double UP_SPEED = 0.4, IN_SPEED = 0.5;
    
    public HopperSubsystem() {
        this.upSpark = new SciSpark(PortMap.HOPPER_ELEVATOR_SPARK);
        this.inSpark = new SciSpark(PortMap.HOPPER_SUCK_SPARK);
    }

    public void setUpSpeed(double speed) { this.upSpark.set(speed); }
    public void setInSpeed(double speed) { this.inSpark.set(speed); }

    public void elevator() { this.setUpSpeed(UP_SPEED); }
    public void suck() { this.setInSpeed(IN_SPEED); }
}