package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.logging.Logger.CommandStatus;

public class IncreaseScaleCommand extends InstantCommand {

    boolean increase;

    public IncreaseScaleCommand(boolean increase) {
        this.increase = increase;
    }

    @Override 
    protected void execute() {
        if(this.increase){
            Robot.driveSubsystem.l.decrementSnapSpeed *= 1.1; 
            Robot.driveSubsystem.r.decrementSnapSpeed *= 1.1;    
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
        } else {
            Robot.driveSubsystem.l.decrementSnapSpeed *= 1/1.1; 
            Robot.driveSubsystem.r.decrementSnapSpeed *= 1/1.1; 
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
            System.out.println("DecrementConstant:" + Robot.driveSubsystem.l.decrementSnapSpeed);                   
        }
    }

}
