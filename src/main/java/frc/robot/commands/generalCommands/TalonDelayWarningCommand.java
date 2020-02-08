package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sciSensorsActuators.SciTalon;
import frc.robot.Utils;

public class TalonDelayWarningCommand extends Command {

    private SciTalon sciTalon;
    private double input;
    private int ticks;

    public TalonDelayWarningCommand (final SciTalon sciTalon, double input) {
        this.sciTalon = sciTalon;
        this.input = input;
        this.ticks = 0;
    }

    @Override 
    protected void execute () {
        ticks++;
        if (!Utils.impreciseEquals(this.sciTalon.getMotorOutputPercent(), this.input)) {
            System.out.println("WARNING: " + this.sciTalon.getDeviceID() + " was set to " + this.input
            + " but still has a value of " + this.sciTalon.getMotorOutputPercent() +" after " + this.ticks + " ticks");
            this.sciTalon.getSciUtils().printAllData();
        }
    }

    @Override
    protected boolean isFinished () { 
        return !Utils.impreciseEquals(this.sciTalon.getMotorOutputPercent(), this.input); 
    }



}