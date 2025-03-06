package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class RemoveAlgaeTop extends SequentialCommandGroup {
    public RemoveAlgaeTop() {
        addCommands(
            new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0),
            new SetElevatorStateTolerance(ElevatorStates.TOPALGAEREMOVAL, 1.5),
            new SetCoralRollersState(CoralRollersState.AlGAEINTAKING)
        );
    }
}
