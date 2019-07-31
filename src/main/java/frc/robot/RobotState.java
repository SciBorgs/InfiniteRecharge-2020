package frc.robot;

import frc.robot.stateEstimation.*;

public class RobotState {
    public updateFunction xUpdater;
    public updateFunction yUpdater;
    public updateFunction angleUpdater;
    public double xPosition;
    public double yPosition;
    public double angle;
    public EncoderLocalization localizer;

    public RobotState() {
        this.xPosition = this.updateX();
        this.yPosition = this.updateY();
        this.angle = this.updateAngle();
        this.localizer = new EncoderLocalization();
        this.xUpdater = ()->this.updateX(); // Lambda expression used to represent function used for this interface
        this.yUpdater = ()->this.updateY();
        this.angleUpdater = ()->this.updateAngle();
    }

    public double updateX(){
        this.localizer.updatePositionTank();
        return this.localizer.getX();
    }

    public double updateY(){
        this.localizer.updatePositionTank();
        return this.localizer.getY();
    }

    public double updateAngle() {
        this.localizer.updatePositionTank();
        return this.localizer.getAngle();
    }
}