package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepSourceCollect extends SequentialCommandGroup{
    public PrepSourceCollect (ElevatorStates elevatorState, WristStates wristState, CoralRollersState coralRollerState, boolean alignToSource) {
        addCommands(
            new SetElevatorState(elevatorState),
            new SetWristState(wristState),
            new SetCoralRollersState(coralRollerState)
        );
    }
}
