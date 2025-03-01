package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.WristStates;


public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral() {
        if (CoralRollers.getInstance().io.HasCoral()){
            addCommands(
                new SetDriveState(DriveState.AlignReef),
                new SetWristStateTolerance(WristStates.PREP,
            0.05,
            ClosedLoopSlot.kSlot0),
                new ParallelCommandGroup(
                    new SetElevatorStateTolerance(manager.level, 1.5),
                    new SetWristStateTolerance(manager.level, 0.001, ClosedLoopSlot.kSlot0)
                ),
                new WaitForAlign(),
                new PlaceCoral(),
                new SetDriveState(DriveState.Reverse)
            );
        }
    }
}
