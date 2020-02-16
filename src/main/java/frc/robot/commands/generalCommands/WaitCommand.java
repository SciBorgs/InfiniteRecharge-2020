package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WaitCommand extends Command {
    private double timeToWait;
    private double startTime;
    private Timer  timer;

    public WaitCommand (double timeToWait) {
        this.timeToWait = timeToWait;
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void initialize() { startTime = timer.get(); }

    @Override
    protected boolean isFinished() { return timer.get() - startTime >= timeToWait; }
}