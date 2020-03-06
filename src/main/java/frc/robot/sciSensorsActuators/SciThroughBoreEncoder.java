package frc.robot.sciSensorsActuators;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.robotState.RobotState.SD;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.sciSensorsActuators.SciThroughBoreEncoder.SciThroughBoreEncoderSD;

public class SciThroughBoreEncoder extends DutyCycleEncoder implements RobotStateUpdater, SciSensorActuator<SciThroughBoreEncoderSD> {
    public static enum SciThroughBoreEncoderSD {Radians}
    public HashMap<SciThroughBoreEncoderSD, SD> sdMap;

    private double offset;

    public SciThroughBoreEncoder(int port) { // NOTE: Must be connected to DIO NOT PWM
        super(port);
        this.sdMap = new HashMap<>();
        this.offset = 0;
        automateStateUpdating();
    }

    public void setOffset(double offset) {this.offset = offset;}
    public void setAngle(double angle)   {this.offset = angle - getDistance();}

    public double getRadians() {return getDistance() + this.offset;}

    @Override
    public HashMap<SciThroughBoreEncoderSD, SD> getSDMap() {return this.sdMap;}

    @Override
    public String getDeviceName() {return "Through Bore Encoder"; }

    @Override
    public void updateRobotState() {
        sciSet(SciThroughBoreEncoderSD.Radians, getRadians());
    }
}