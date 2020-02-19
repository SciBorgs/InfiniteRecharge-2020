package frc.robot.commands.twister;

import frc.robot.Robot;
import frc.robot.helpers.DelayedPrinter;
import frc.robot.logging.Logger.CommandStatus;
import frc.robot.subsystems.TwisterSubsystem.Color;
import frc.robot.colors.*;
import frc.robot.robotState.RobotState.SD;
import edu.wpi.first.wpilibj.command.Command;

public class RotateControlPanelCommand extends Command {
    private static final int STATES_AGO = 5;
    private int timesToTurn;
    private int elapsedTurns;
    // private Color startingColor;
   
    
    public RotateControlPanelCommand(int times){
        requires(Robot.twisterSubsystem);
        this.timesToTurn = times;
        this.elapsedTurns = 0;
    }

    @Override
    protected void initialize() {
        // this.startingColor = Robot.twisterSubsystem.getColor();
    }

    @Override 
    protected void execute() {
        Robot.twisterSubsystem.startWheel();
    }

    @Override
    protected boolean isFinished() {
        // if (Robot.twisterSubsystem.getColor() != this.startingColor) {
        if (Robot.twisterSubsystem.didColorChange(STATES_AGO)) {
            elapsedTurns++;
        }
        return elapsedTurns == timesToTurn;
    }

    @Override
    protected void end() {
        Robot.twisterSubsystem.stopWheel();
    }
}
