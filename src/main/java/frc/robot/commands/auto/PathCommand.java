package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.autoProfiles.Path;
import frc.robot.autoProfiles.Segment;

public class PathCommand extends Command {
    
    private ArrayList<SegmentCommand> remainingSegments;
    private SegmentCommand currSegmentCommand;

    public PathCommand (Path path) {
        remainingSegments = new ArrayList<>(path.sequence.size());
        for (Segment segment : path.sequence) {
            remainingSegments.add(new SegmentCommand(segment));
        }
        currSegmentCommand = null;
    }

    @Override
    protected void execute() {
        if (currSegmentCommand == null) {
            if (remainingSegments.isEmpty()) {
                return;
            }
            currSegmentCommand = remainingSegments.remove(0);
            currSegmentCommand.start();
        }
        currSegmentCommand.execute();

        if (currSegmentCommand.isCompleted()) {
            currSegmentCommand = null;
        }

    }

    @Override
    protected boolean isFinished() {
        return remainingSegments.isEmpty() && currSegmentCommand == null;
    }
}