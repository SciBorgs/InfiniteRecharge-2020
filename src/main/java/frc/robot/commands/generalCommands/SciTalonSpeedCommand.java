package frc.robot.commands.generalCommands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sciSensorsActuators.SciTalon;

public class SciTalonSpeedCommand extends CommandBase {

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
    public void execute(){this.talon.instantSet();}
    @Override
    public boolean isFinished(){
        return this.talon.atGoal() || !this.talon.isCurrentCommandNumber(this.commandNunber);
    }

}