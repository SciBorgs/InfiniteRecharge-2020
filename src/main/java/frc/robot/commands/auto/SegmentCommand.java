package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.autoProfiles.Segment;

public class SegmentCommand extends Command {

    private Segment segment;
    private CircleControllerCommand circleControllerCommand;

    private ArrayList<Command> remainingCommands;
    private Command currCommand;

    public SegmentCommand (Segment segment) {
        this.segment = segment;
        this.circleControllerCommand = new CircleControllerCommand(this.segment.waypoint);
        Robot.driveSubsystem.setReveresed(this.segment.reverse);
        remainingCommands = new ArrayList<>();
        for (Command command : segment.commands) {
            remainingCommands.add(command);
        }
        currCommand = null;
    }

    @Override
    protected void execute() {
        if (currCommand == null) {
            if (remainingCommands.isEmpty()) {
                return;
            }
            if (remainingCommands.get(0) == segment.parallelCommand) {
                circleControllerCommand.start();
            } else if (circleControllerCommand.isCompleted()) {
                currCommand = null;
            }
            currCommand = remainingCommands.remove(0);
            currCommand.start();
        }

        if (currCommand.isCompleted()) {
            currCommand = null;
        }
    }    

    @Override
    protected boolean isFinished() {
        return remainingCommands.isEmpty() && currCommand == null;
    }
}