package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.sciSensorsActuators.SciSpark;


public class SparkDelayWarningCommand extends InstantCommand {

    private long delay = 4; // 50 ticks/sec
    private SciSpark sciSpark;
    private double limitedInput;
    private Thread t;

    public SparkDelayWarningCommand(final SciSpark sciSpark, double limitedInput) {
        this.sciSpark = sciSpark;
        this.limitedInput = limitedInput;
    }

    @Override 
    protected void execute() {
        try {
            t.wait(delay);
        } catch (InterruptedException e) {
            System.out.println("WARNING: " + this.sciSpark.getDeviceName() + " was set to " + this.limitedInput
            + " but still has a value of " + this.sciSpark.get());
        }
    }
}