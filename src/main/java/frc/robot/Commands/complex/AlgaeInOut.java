package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class AlgaeInOut extends SequentialCommandGroup{
    public AlgaeInOut(CoralRollersState state){
        addCommands(
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(state);
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
            })
        );
    }
}
