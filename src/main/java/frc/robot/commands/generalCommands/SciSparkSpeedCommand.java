package frc.robot.commands.generalCommands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sciSensorsActuators.SciSpark;

public class SciSparkSpeedCommand extends CommandBase {

    private SciSpark spark;
    private double goalSpeed;
    private double maxJerk;
    private int commandNunber;

    public SciSparkSpeedCommand(SciSpark spark, int commandNumber){
        this.spark = spark;
        this.commandNunber = commandNumber;
        this.goalSpeed = this.spark.goalSpeed;
        this.maxJerk = this.spark.currentMaxJerk;
    }

    @Override
    public void execute(){this.spark.instantSet();}
    @Override
    public boolean isFinished(){return this.spark.atGoal();}

}