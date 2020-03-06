package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ElevatorHopperCommand extends InstantCommand {

    @Override
    protected void execute(){
        Robot.hopperSubsystem.elevator();
    }
}