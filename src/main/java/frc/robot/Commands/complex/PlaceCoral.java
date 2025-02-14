package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class PlaceCoral extends SequentialCommandGroup {
    public PlaceCoral() {
        addCommands(
                new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
                }),
                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                }));
    }
}
