package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.SetTargetReefSide;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.WristStates;

public class ClearAlgae extends SequentialCommandGroup {
    public ClearAlgae() {
        addCommands(
                new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0),
                new ConditionalCommand(
                        new RemoveAlgaeTop(),
                        new RemoveAlgaeBottom(),
                        () -> {
                            ReefTargetOrientation targetOrientation = Swerve.getInstance().getReefTargetOrientation();
                            return (targetOrientation == ReefTargetOrientation.AB || targetOrientation == ReefTargetOrientation.EF
                                    || targetOrientation == ReefTargetOrientation.IJ);
                        }),
                new SetDriveState(DriveState.AlignReef),
                new WaitCommand(0.2),
                new WaitUntilCommand(() -> CoralRollers.getInstance().isStalled()),

                new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Reverse);
                }),
                new SetElevatorState(ElevatorStates.LOW),
                new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),
                new WaitCommand(0.1),
                new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Manual);
                }));
    }
}
