package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeTop;
import frc.robot.Commands.complex.RemoveAlgaeBottom;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class RemoveAlgaeInAuto extends SequentialCommandGroup {
    public RemoveAlgaeInAuto(ReefTargetOrientation targetOrientation) {
        BooleanSupplier isTop = () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF
                || targetOrientation == States.ReefTargetOrientation.IJ;
        addCommands(
            new DriveBack().withTimeout(0.4),
            new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0),
            new ConditionalCommand(
                    new TestRemoveAlgaeTop(),
                    new RemoveAlgaeBottom(),
                    isTop),
            new PlaceAlgaeInAuto(targetOrientation),
            new SetElevatorState(ElevatorStates.LOW),
            new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),

            new DriveBack().withTimeout(0.5)
        );
    }
}