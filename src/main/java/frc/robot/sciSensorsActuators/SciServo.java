package frc.robot.sciSensorsActuators;

import java.util.Optional;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Robot;
import frc.robot.robotState.RobotState.SD;

public class SciServo extends Servo {
    private static final double ANGLE_RANGE = Math.PI;
    private double minAngle = 0;
    public Optional<SD> angleSD, rawSD;

    public SciServo(int channel) {
        super(channel);
        this.minAngle = 0;
    }

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
        Robot.optionalSet(this.angleSD, getAngle());
        Robot.optionalSet(this.rawSD,   super.get());
    }
    
    public void assignAngleSD(SD angleSD) {this.angleSD = Optional.of(angleSD);}
    public void assignRawSd  (SD rawSD)   {this.rawSD   = Optional.of(rawSD);}
}
