package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sciSensorsActuators.SciSpark;

public class SciSparkSpeedCommand extends Command {

    private SciSpark spark;
    private int commandNunber;

    public SciSparkSpeedCommand(SciSpark spark, int commandNumber){
        this.spark = spark;
        this.commandNunber = commandNumber;
    }

    @Override
    protected void execute(){this.spark.instantSet();}
    @Override
    protected boolean isFinished(){
        return this.spark.atGoal() || !this.spark.isCurrentCommandNumber(this.commandNunber);
    }

}