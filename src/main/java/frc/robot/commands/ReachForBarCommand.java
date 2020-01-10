package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Command;

public class ReachForBarCommand extends Command {
    private final double ERROR = 0.05;
    private final double BAR_HEIGHT = Utils.inchesToMeters(63);
    
    public ReachForBarCommand(){}

    @Override
    protected void execute(){
        Robot.climberController.moveToHeight(BAR_HEIGHT);
    } 

    @Override
    protected boolean isFinished() {
        return Utils.inRange(Robot.get(SD.ClimberHeight), BAR_HEIGHT, ERROR);
    }

    @Override
    protected void end(){
        Robot.climberSubsystem.setLiftSpeed(0);
    }

}