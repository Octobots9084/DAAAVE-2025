package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class BargeThrow extends SequentialCommandGroup{
    public BargeThrow(){
        addCommands(
            new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),
            new SetElevatorState(ElevatorStates.LEVEL4),       
            new WaitCommand(0.7),
            new SetWristState(WristStates.BARGEALGAE,ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.AlGAEOUTPUT)
        );
    }
}
