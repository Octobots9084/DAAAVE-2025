package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class ReverseRollersWileTrue extends Command {
    public void initialize()
    {
        CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
    }

    public void end(boolean interrupted)
    {
        CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
    }
}
