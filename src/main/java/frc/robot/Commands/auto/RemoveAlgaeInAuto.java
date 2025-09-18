package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeBottom;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeTop;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class RemoveAlgaeInAuto extends SequentialCommandGroup {
    public RemoveAlgaeInAuto(ReefTargetOrientation targetOrientation) { 
        BooleanSupplier isTop =  () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;

        addCommands(
            new ParallelCommandGroup(
                new AlignInAuto(ReefTargetSide.ALGAE, targetOrientation),
                new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0)
            ),
            
            new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new SetElevatorState(ElevatorStates.BrazillianCycle),
                    new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),
                    isTop
                ),
                new ConditionalCommand(
                    new SetWristState(WristStates.L3, ClosedLoopSlot.kSlot0),
                    new SetWristState(WristStates.ALAGESTACKREMOVAL, ClosedLoopSlot.kSlot0),
                    isTop
                )
            ),
            
            new WaitUntilCommand(() -> CoralRollers.getInstance().isStalled()),
            
            new DriveBack().withTimeout(0.2),
            // new ParallelCommandGroup(
                // new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),
                new SetElevatorState(ElevatorStates.LOW)
            // )
        );
    }
}