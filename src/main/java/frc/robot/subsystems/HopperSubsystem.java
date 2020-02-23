package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.PortMap;

public class HopperSubsystem {
    public SciSpark upSpark, inSpark;
    private static final double HOPPER_SPEED = 0.3;
    
    public HopperSubsystem() {
        this.upSpark = new SciSpark(PortMap.HOPPER_UP_SPARK);
        this.inSpark = new SciSpark(PortMap.HOPPER_IN_SPARK);
    }

    public void setUpSpeed(double speed) { this.upSpark.set(speed); }
    public void setInSpeed(double speed) { this.inSpark.set(speed); }

    public void setUpSpeed() { this.setUpSpeed(HOPPER_SPEED); }
    public void setInSpeed() { this.setInSpeed(HOPPER_SPEED); }
}