package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.helpers.Geo;
import frc.robot.robotState.RobotState.SD;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.command.Command;

public class AdjustAngleCommand extends Command {
    private double targetAngle;
    private final String FILENAME = "AdjustAngleCommand.java";

    public AdjustAngleCommand() {}
    

    protected void execute() {
        if (Math.abs(Geo.subtractAngles(-3/8 * Math.PI, Robot.get(SD.Angle))) < 1/2 * Math.PI){
            targetAngle = -3/8 * Math.PI;
        } else {
            targetAngle = 5/8 * Math.PI;
        }
        Robot.angleController.goToAngle(targetAngle);
    }

    protected boolean isFinished() {
        
        return Utils.inRange(Robot.get(SD.Angle), targetAngle, Math.toRadians(1));
    }
}