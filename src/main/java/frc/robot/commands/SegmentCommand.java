package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.autoProfiles.Segment;


public class SegmentCommand extends Command {

    private Segment segment;
    private double startWait, endWait;
    private WaitCommand startWaitCommand, endWaitCommand;
    private boolean seqCommandStarted, parCommandStarted;
    private CircleControllerCommand circleControllerCommand;

    public SegmentCommand (Segment segment) {
        this.segment = segment;
        this.startWait = this.segment.startWait;
        this.startWaitCommand = new WaitCommand(startWait);
        this.endWaitCommand = new WaitCommand(endWait);
        this.circleControllerCommand = new CircleControllerCommand(this.segment.waypoint);
    }

    @Override
    public synchronized void start() {
        startWaitCommand.start();
    }

    @Override
    protected void execute() {
        if (startWaitCommand.isCompleted() && !this.seqCommandStarted){
            this.segment.sequentialCommand.start();
            this.seqCommandStarted = true;
        } else if (startWaitCommand.isCompleted() && !this.parCommandStarted && this.segment.sequentialCommand.isCompleted()) {
            this.segment.parallelCommand.start();
            this.circleControllerCommand.start();
            this.parCommandStarted = true;
        } else if (this.circleControllerCommand.isCompleted()) {
            this.segment.doneCommand.start();
        } else if (this.segment.doneCommand.isCompleted()) {
            this.endWaitCommand.start();
        }
    }    

    @Override
    protected boolean isFinished() {
        return this.endWaitCommand.isCompleted(); 
    }
}