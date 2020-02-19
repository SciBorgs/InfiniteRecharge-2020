package frc.robot.commands.twister;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TurnControlPanel3RevsCommand extends CommandGroup {
    private static final double PISTON_WAIT_TIME = 0.1;
    
    public TurnControlPanel3RevsCommand(){
        addSequential(new ToggleTwisterPistonCommand());
        addSequential(new WaitCommand(PISTON_WAIT_TIME));
        addSequential(new RotateControlPanelCommand(24));
        addSequential(new ToggleTwisterPistonCommand());
    }
}
