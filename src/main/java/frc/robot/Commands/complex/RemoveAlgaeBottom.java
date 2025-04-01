package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class RemoveAlgaeBottom extends SequentialCommandGroup {
    public RemoveAlgaeBottom() {
        addCommands(
                new ConditionalCommand(new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0), new InstantCommand(), () -> {
                    return Wrist.getInstance().IsInsideRobot();
                }),
                new InstantCommand(() -> AlignVision.setPoleSide(ReefTargetSide.ALGAE)),
                new SetElevatorStateTolerance(ElevatorStates.BOTTOMALGAE, 1.5),
                new SetWristStateTolerance(WristStates.ALGAEREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
                new SetCoralRollersState(CoralRollersState.ALGAEINTAKING));
    }
}
