package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class SuckHopperCommand extends InstantCommand {

    @Override
    protected void execute(){
        Robot.hopperSubsystem.suck();
    }

}