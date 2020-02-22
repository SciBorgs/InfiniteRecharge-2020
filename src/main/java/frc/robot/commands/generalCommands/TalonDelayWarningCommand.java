package frc.robot.commands.generalCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sciSensorsActuators.SciTalon;
import frc.robot.Utils;

public class TalonDelayWarningCommand extends CommandBase {

    private SciTalon sciTalon;
    private double input;
    private int ticks;

    public TalonDelayWarningCommand (final SciTalon sciTalon, double input) {
        this.sciTalon = sciTalon;
        this.input = input;
        this.ticks = 0;
    }

    @Override 
    public void execute () {
        ticks++;
        if (!Utils.impreciseEquals(this.sciTalon.getMotorOutputPercent(), this.input)) {
            System.out.println("WARNING: " + this.sciTalon.getDeviceID() + " was set to " + this.input
            + " but still has a value of " + this.sciTalon.getMotorOutputPercent() +" after " + this.ticks + " ticks");
            this.sciTalon.printAllData();
        }
    }

    @Override
    public boolean isFinished () { 
        return !Utils.impreciseEquals(this.sciTalon.getMotorOutputPercent(), this.input); 
    }



}