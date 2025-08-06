package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class Intake extends SequentialCommandGroup {
    public Intake() {

        addCommands(
            new PrepCollect().andThen(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new CollectCoral(),
                        new ConditionalCommand(new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0), new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0), () -> manager.level == ElevatorStates.LEVEL1)
                    ),
                    new InstantCommand(), () -> {return Wrist.getInstance().isAtState(WristStates.INTAKE, 0.2);})
            )
        );
    }
}
