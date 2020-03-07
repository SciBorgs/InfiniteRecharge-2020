package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.logging.Logger.CommandStatus;
import frc.robot.subsystems.IntakeSubsystem.IntakeValue;

public class IntakeSuckCommand extends InstantCommand {

    @Override 
    protected void execute() {
	    Robot.logger.logCommandStatus(CommandStatus.Executing);
        Robot.intakeSubsystem.suck();
        //Robot.intakeSubsystem.upDownSolenoid.set(IntakeValue.Down);
    }
    
}
