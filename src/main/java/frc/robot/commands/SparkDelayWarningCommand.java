package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.sciSensorsActuators.SciSpark;
import edu.wpi.first.wpilibj.Timer;

public class SparkDelayWarningCommand extends InstantCommand {

    private long delay = 10; // in milliseconds
    private SciSpark sciSpark;
    private double limitedInput;
    private Timer timer;
    public SparkDelayWarningCommand(final SciSpark sciSpark, double limitedInput) {
        this.sciSpark = sciSpark;
        this.limitedInput = limitedInput;
		this.timer = new Timer();
        this.timer.start();   
    }

    @Override 
    protected void execute() {
        if (timer.get() == delay) {
        System.out.println("WARNING: " + this.sciSpark.getDeviceName() + " was set to " + this.limitedInput
            + " but still has a value of " + this.sciSpark.get());
            System.out.println("interreupted");
        }
    }
}