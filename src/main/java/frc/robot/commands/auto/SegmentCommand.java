package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.autoProfiles.Segment;
import frc.robot.commands.generalCommands.WaitCommand;

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
            if (Utils.hasCommandStarted(this.segment.sequentialCommand)){
                this.segment.sequentialCommand.start();
            } else if (!this.circleControllerCommand.isCompleted() && this.segment.sequentialCommand.isCompleted()) {
                if (Utils.hasCommandStarted(this.segment.parallelCommand)) {
                    this.segment.parallelCommand.start();
                    this.circleControllerCommand.start();
                } else if (Utils.hasCommandStarted(this.segment.doneCommand)) {
                    this.segment.doneCommand.start();
                }
            } else if (this.segment.doneCommand.isCompleted() && Utils.hasCommandStarted(this.endWaitCommand)) {
                this.endWaitCommand.start();
            }
        }
    }    

    @Override
    protected boolean isFinished() {
        return this.endWaitCommand.isCompleted();
    }
}