package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.PortMap;

public class HopperSubsystem {
    
    public SciSpark elevatorSpark, suckSpark;
    private static final double ELEVATOR_SPEED = 0.4, SUCK_SPEED = 0.5;
    
    public HopperSubsystem() {
        this.elevatorSpark = new SciSpark(PortMap.HOPPER_ELEVATOR_SPARK);
        this.suckSpark = new SciSpark(PortMap.HOPPER_SUCK_SPARK);
    }

    public void setElevatorSpeed(double speed) { this.elevatorSpark.set(speed); }
    public void setSuckSpeed(double speed)     { this.suckSpark.set(speed); }

    public void elevator() { this.setElevatorSpeed(ELEVATOR_SPEED); }
    public void suck() { this.setSuckSpeed(SUCK_SPEED); }
}