package frc.robot.commands.twister;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleTwisterPistonCommand extends InstantCommand {
    @Override 
    protected void execute() {
        Robot.twisterSubsystem.toggleDoubleSolenoid();
    }   
}
