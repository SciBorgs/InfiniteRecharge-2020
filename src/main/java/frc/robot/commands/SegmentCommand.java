package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.autoProfiles.Segment;


public class SegmentCommand extends Command {

    private Segment segment;
    private WaitCommand startWaitCommand, endWaitCommand;
    private CircleControllerCommand circleControllerCommand;

    public SegmentCommand (Segment segment) {
        this.segment = segment;
        this.startWaitCommand = new WaitCommand(this.segment.startWait);
        this.endWaitCommand = new WaitCommand(this.segment.endWait);
        this.circleControllerCommand = new CircleControllerCommand(this.segment.waypoint);
        Robot.driveSubsystem.setReveresed(this.segment.reverse);
    }

    @Override
    public synchronized void start() {
        startWaitCommand.start();
    }

    @Override
    protected void execute() {
        if (this.startWaitCommand.isCompleted()) {
            if (!this.segment.sequentialCommand.isRunning() && !this.segment.sequentialCommand.isCompleted()){
                this.segment.sequentialCommand.start();
            } else if (!this.segment.parallelCommand.isRunning() && this.segment.sequentialCommand.isCompleted()) {
                this.segment.parallelCommand.start();
                this.circleControllerCommand.start();
                System.out.println("currentState: " + circleControllerCommand.circleController.isFinished());
            } else if (!this.circleControllerCommand.isCompleted() && !this.segment.doneCommand.isRunning()) {
                this.segment.doneCommand.start();
            } else if (this.segment.doneCommand.isCompleted()) {
                this.endWaitCommand.start();
            }
        }
    }    

    @Override
    protected boolean isFinished() {
        return this.endWaitCommand.isCompleted(); 
    }
}