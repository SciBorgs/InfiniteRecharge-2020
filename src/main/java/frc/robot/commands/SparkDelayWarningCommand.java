package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.Utils;

public class SparkDelayWarningCommand extends Command {

    private SciSpark sciSpark;
    private double input;
    private int ticks;

    public SparkDelayWarningCommand (final SciSpark sciSpark, double input) {
        this.sciSpark = sciSpark;
        this.input = input;
        ticks = 0;
    }

    @Override 
    protected void execute () {
        ticks++;
        if (!Utils.impreciseEquals(this.sciSpark.get(), this.input)) {
            System.out.println("WARNING: " + this.sciSpark.getDeviceName() + " was set to " + this.input
            + " but still has a value of " + this.sciSpark.get() +" after " + ticks + " ticks");
        }
    }

    @Override
    protected boolean isFinished () { 
        return !Utils.impreciseEquals(this.sciSpark.get(), this.input); 
    }



}