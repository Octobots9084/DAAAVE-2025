package frc.robot.Commands.auto.testing.Algae;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class TestRemoveAlgaeTop extends SequentialCommandGroup {
    public TestRemoveAlgaeTop() {
        addCommands(
                new ConditionalCommand(new SetWristStateTolerance(WristStates.TOPALGAEREMOVAL, 0.05, ClosedLoopSlot.kSlot0), new InstantCommand(), () -> {
                    return Wrist.getInstance().IsInsideRobot();
                }),
                new SetElevatorStateTolerance(ElevatorStates.TOPALGAE, 1.5),
                new SetWristStateTolerance(WristStates.TOPALGAEREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
                new SetCoralRollersState(CoralRollersState.ALGAEINTAKING));
    }
}