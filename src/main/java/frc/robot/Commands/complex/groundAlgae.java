package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetAlgaeRollerState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class groundAlgae extends SequentialCommandGroup{
    public groundAlgae(){
        addCommands(
            new SetElevatorState(ElevatorStates.LOW),
            new WaitCommand(0.2),
            new SetWristState(WristStates.GROUNDALGAE, ClosedLoopSlot.kSlot0),
            new SetAlgaeRollerState(CoralRollersState.ALGAEINTAKING)
        );
    }

}
