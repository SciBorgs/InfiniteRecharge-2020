package frc.robot.commands.twister;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TurnControlPanelOnceCommand extends CommandGroup {
    private static final double PISTON_WAIT_TIME = 0.1;
    
    public TurnControlPanelOnceCommand(){
        addSequential(new ToggleTwisterPistonCommand());
        addSequential(new WaitCommand(PISTON_WAIT_TIME));
        addSequential(new RotateControlPanelCommand(1));
        addSequential(new ToggleTwisterPistonCommand());
    }
}
