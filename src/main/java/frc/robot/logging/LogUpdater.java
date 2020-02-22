package frc.robot.logging;

import frc.robot.Robot;

public interface LogUpdater {

    public void periodicLog();
    default void automateLogging(){
        Robot.addLogUpdater(this);
    }
}