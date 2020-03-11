package frc.robot.subsystems;

import frc.robot.sciSensorsActuators.SciSpark;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.RobotState.SD;

public class HopperSubsystem implements RobotStateUpdater {

    public SciSpark elevatorSpark, suckSpark;
    private static final double ELEVATOR_SPEED = 0.4, SUCK_SPEED = 0.5;
    private Rev2mDistanceSensor distanceSensor;

    public HopperSubsystem() {
        this.elevatorSpark = new SciSpark(PortMap.HOPPER_ELEVATOR_SPARK);
        this.suckSpark = new SciSpark(PortMap.HOPPER_SUCK_SPARK);
        this.distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        automateStateUpdating();
    }

    @Override
    public void updateRobotState() {
        double range = distanceSensor.getRange(Unit.kMillimeters);
        Robot.setMapped(RobotState.BOOLEAN_MAPPING, SD.BallInElevator, range < 30 ? true : false);
    }

    public void setElevatorSpeed(double speed) { this.elevatorSpark.set(speed); }
    public void setSuckSpeed(double speed)     { this.suckSpark.set(speed); }

    public void elevator() { this.setElevatorSpeed(ELEVATOR_SPEED); }
    public void suck() { this.setSuckSpeed(SUCK_SPEED); }
    public void stopElevator() {this.setElevatorSpeed(0);}
    public void stopSuck()     {this.setSuckSpeed(0);}

}