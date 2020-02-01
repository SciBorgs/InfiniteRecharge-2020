package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sciSensorsActuators.SciSpark;

public class SciSparkSpeedCommand extends Command {

    private SciSpark spark;

    public SciSparkSpeedCommand(SciSpark spark){
        this.spark = spark;
    }

    @Override
    protected void execute(){this.spark.moveToGoal();}
    @Override
    protected boolean isFinished(){return this.spark.atGoal();}

}