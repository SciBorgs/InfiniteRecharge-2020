package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.command.Command;

public class EmptyCommand extends Command {

    public EmptyCommand () {
        System.out.println("make wait command");
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }
}