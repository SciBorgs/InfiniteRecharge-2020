package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;

public class IntakeSuckCommand extends Command {
    private Timer   timer;
    private double  timeToWait;
    private double  startTime;
    private boolean usingTimeout;
    
    public IntakeSuckCommand() {
        requires(Robot.intakeSubsystem);
    }

    public IntakeSuckCommand(double timeout) { // timeout in seconds
        this();
        requires(Robot.intakeSubsystem);
        this.timeToWait = timeout;
        usingTimeout = true;
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void initialize() { 
        if (usingTimeout) {
            startTime = timer.get(); 
        }
    }

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.suck();
    }
    
    @Override
    protected boolean isFinished() {
        return usingTimeout &&  timer.get() - startTime >= timeToWait;
    }

    @Override
    protected void end() {
        if (usingTimeout) {
            Robot.intakeSubsystem.stop();
        }
    }
    
}
