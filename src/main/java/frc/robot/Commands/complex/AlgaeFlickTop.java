package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.WristStates;

public class AlgaeFlickTop extends SequentialCommandGroup {
    public AlgaeFlickTop() {
        addCommands(new InstantCommand(() -> AlignVision.setPoleSide(ReefTargetSide.ALGAE)),
                new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0), new SetElevatorStateTolerance(ElevatorStates.TOPALGAE, 3),
                new SetWristState(WristStates.QUICKALGAEREMOVALHIGH, ClosedLoopSlot.kSlot0),
                new SetCoralRollersState(CoralRollersState.AlGAEOUTPUT));
    }
}
