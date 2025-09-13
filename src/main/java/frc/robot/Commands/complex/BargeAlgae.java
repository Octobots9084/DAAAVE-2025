package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class BargeAlgae extends SequentialCommandGroup{
    public BargeAlgae(){
        addCommands(
            new SetElevatorState(ElevatorStates.LEVEL4),
            new SetWristState(WristStates.BARGEALGAE, ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.AlGAEOUTPUT)
        );
    }
}
