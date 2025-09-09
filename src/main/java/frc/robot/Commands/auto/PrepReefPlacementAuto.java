package frc.robot.Commands.auto;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepReefPlacementAuto extends InstantCommand {

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new SetWristStateTolerance(WristStates.PREP,
                        0.05,
                        ClosedLoopSlot.kSlot0),
                new SetElevatorStateTolerance(manager.level, 5).withTimeout(20),
                new SetWristStateTolerance(manager.level, 0.01, ClosedLoopSlot.kSlot0)));
    }
}
