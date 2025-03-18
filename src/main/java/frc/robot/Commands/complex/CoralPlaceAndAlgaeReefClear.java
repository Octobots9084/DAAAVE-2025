package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class CoralPlaceAndAlgaeReefClear extends SequentialCommandGroup{
    public CoralPlaceAndAlgaeReefClear(){
        ElevatorStates isTop = (Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.AB || Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.EF || Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.IJ) ? ElevatorStates.TOPALGAE : ElevatorStates.BOTTOMALGAE;
        addCommands(
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.AlignReef);
            }),
            new ScoreCoral(),
            new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Reverse);
                }),
            new WaitCommand(.25),
            new SetElevatorState(isTop),
            new WaitCommand(0.15),
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.Manual);
            }),
            new ClearAlgae()
        );
    }
}
