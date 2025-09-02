package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeBottom;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeTop;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class RemoveAlgaeInAuto extends SequentialCommandGroup {
    public RemoveAlgaeInAuto(ReefTargetOrientation targetOrientation) { 
        BooleanSupplier isTop =  () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;

        addCommands(
            new AlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, targetOrientation),

            new SetWristStateTolerance(WristStates.ALGAEREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            new ConditionalCommand(
                new SetElevatorState(ElevatorStates.TOPALGAEFAST),
                new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),
                isTop
            ),
            // new WaitCommand(0.45),//make conditional; if top, its less time, if bottom, more to go down more
            new ConditionalCommand(
                new WaitCommand(0.3),
                new WaitCommand(0.4),
                isTop),
            new ParallelCommandGroup(
                new DriveBack().withTimeout(0.5),
                new SetElevatorStateTolerance(ElevatorStates.LOW, 0.05),
                new SetWristState(WristStates.PREP,ClosedLoopSlot.kSlot0)
            ).withTimeout(1)
        );
    }
}