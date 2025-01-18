package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepReefPlacement extends SequentialCommandGroup{
    public PrepReefPlacement (ElevatorStates elevatorState, WristStates wristState, boolean reefAlignRight) {
        addCommands(
            new SetElevatorState(elevatorState),
            new SetWristState(wristState)
        );
    }
}
