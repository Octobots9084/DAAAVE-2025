package frc.robot.Commands.Emote;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.complex.Intake;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class BrazilianCycle extends SequentialCommandGroup {
    public BrazilianCycle() {
        addCommands(
            new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0).withTimeout(0.5),
            new ParallelCommandGroup(
                new SetWristState(WristStates.BrazilianCycle2, ClosedLoopSlot.kSlot0).withTimeout(0.5),
                new SetElevatorState(ElevatorStates.BrazillianCycle)
            ),
            new SetWristState(WristStates.BrazilianCycle3, ClosedLoopSlot.kSlot0).withTimeout(0.5),
            new ParallelCommandGroup(
                new SetCoralRollersState(CoralRollersState.BrazilianCycle),
                new SetElevatorState(ElevatorStates.LOW),
                new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0)
            ),
            new Intake()
            //olier
            // new SetWristState(WristStates.BrazilianCycle3, ClosedLoopSlot.kSlot0),
            
            // new ParallelCommandGroup(
            //     new SetElevatorState(ElevatorStates.LEVEL2),
            //     new SetCoralRollersState(CoralRollersState.BrazilianCycle)
            // ).withTimeout(5),
            // new SetElevatorState(ElevatorStates.LOW),
            // // new ParallelRaceGroup(
            //     // new Intake()
            // // ),
            // new Intake()
            //michael
        );
    }
}
