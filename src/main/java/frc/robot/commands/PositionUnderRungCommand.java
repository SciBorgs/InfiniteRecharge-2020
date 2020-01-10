package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PositionUnderRungCommand extends CommandGroup {
    private final String FILENAME = "ActivateGeneratorCommand.java";

    public PositionUnderRungCommand() {
        requires(Robot.climberSubsystem);
        addSequential(new AdjustAngleCommand());
        addSequential(new ReachForBarCommand());
    }

    @Override
    protected void execute() {}
}