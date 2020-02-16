package frc.robot.sciSensorsActuators;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Robot;
import frc.robot.robotState.RobotStateUpdater;
import frc.robot.robotState.RobotState.SD;
import frc.robot.sciSensorsActuators.SciServo.SciServoSD;

public class SciServo extends Servo implements RobotStateUpdater, SciSensorActuator<SciServoSD> {
    public static enum SciServoSD {Angle, Raw}
    public HashMap<SciServoSD, SD> sdMap;
    private static final double ANGLE_RANGE = Math.PI;
    private double minAngle = 0;

    public SciServo(int channel) {
        super(channel);
        automateStateUpdating();
    }

    @Override
    public HashMap<SciServoSD, SD> getSDMap(){return this.sdMap;}
    @Override 
    public String getDeviceName(){return "Servo " + super.getChannel();}

    public double getMinAngle()    { return this.minAngle; }
    public double getCenterAngle() { return this.minAngle + ANGLE_RANGE/2; }
    public double getMaxAngle()    { return this.minAngle + ANGLE_RANGE; }

    public void setMinAngle(double angle)    { this.minAngle = angle; }
    public void setCenterAngle(double angle) { this.minAngle = angle - ANGLE_RANGE/2.0; }
    public void setMaxAngle(double angle)    { this.minAngle = angle - ANGLE_RANGE; }

    @Override
    public double getAngle() {
        return Math.toRadians(super.getAngle()) + this.minAngle;
    }

    @Override
    public void setAngle(double angle) {
        super.setAngle(Math.toDegrees(angle - this.minAngle));
    }

    public void updateRobotState() {
        sciSet(SciServoSD.Angle, getAngle());
        sciSet(SciServoSD.Raw,   super.get());
    }
}
