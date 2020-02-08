package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sciSensorsActuators.SciTalon;

public class SciTalonSpeedCommand extends Command {

    private SciTalon talon;
    private double goalSpeed;
    private double maxJerk;
    private int commandNunber;

    public SciTalonSpeedCommand(SciTalon talon, int commandNumber){
        this.talon = talon;
        this.commandNunber = commandNumber;
        this.goalSpeed = this.talon.goalSpeed;
        this.maxJerk = this.talon.currentMaxJerk;
    }

    @Override
    protected void execute(){this.talon.instantSet();}
    @Override
    protected boolean isFinished(){
        return this.talon.atGoal() || !this.talon.isCurrentCommandNumber(this.commandNunber);
    }
    @Override
    protected void end(){ }

}