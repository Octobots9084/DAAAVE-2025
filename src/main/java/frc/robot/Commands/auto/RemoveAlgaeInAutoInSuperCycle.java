package frc.robot.Commands.auto;

import java.io.SequenceInputStream;
import java.util.function.BooleanSupplier;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class RemoveAlgaeInAutoInSuperCycle extends SequentialCommandGroup{
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;

    public RemoveAlgaeInAutoInSuperCycle(ReefTargetOrientation targetOrientation, ReefTargetSide targetSide) {
        this.targetOrientation = targetOrientation;
        this.targetSide = targetSide;
        BooleanSupplier isTop =  () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;

        addCommands(
            new AlignInAuto(ReefTargetSide.ALGAE, targetOrientation),
            new SetWristStateTolerance(WristStates.ALAGESTACKREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            // new ParallelCommandGroup(
            new ConditionalCommand(
                new SetElevatorState(ElevatorStates.TOPALGAEFAST),
                new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),
                isTop
            ),//NEED TO GO DOWN
            // new WaitCommand(0.45),
            // new ConditionalCommand(
            //     new WaitCommand(0.3),
            //     new WaitCommand(0.4),
            //     isTop),
            // // ),
            // new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            // new WaitCommand(0.5),
            // new ConditionalCommand(
            //     new SetElevatorState(ElevatorStates.TOPALGAEFAST),
            //     new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),
            //     isTop
            // ),//NEED TO GO DOWN
            // new WaitCommand(0.45),
            // new ConditionalCommand(
            //     new WaitCommand(0.3),
            //     new WaitCommand(0.4),
            //     isTop),
            new WaitUntilCommand(() -> CoralRollers.getInstance().isStalled()),
            // new ParallelCommandGroup(
                // new DriveBack().withTimeout(0.5),
                new DriveBack().withTimeout(0.3)
                // new SetElevatorStateTolerance(ElevatorStates.LOW, 0.05),
                // new SetWristState(WristStates.PREP,ClosedLoopSlot.kSlot0)
            // ).withTimeout(1)
        );
    }
}