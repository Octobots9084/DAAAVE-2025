package frc.robot.Commands.auto.testing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;

public class TestAlignAnyInAuto extends ParallelCommandGroup{
    public TestAlignAnyInAuto (ReefTargetSide targetSide) {
        addCommands(
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.AB),
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.CD),
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.EF),
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.GH),
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.IJ),
            new TestAlignInAuto(ElevatorStates.LOW, targetSide, ReefTargetOrientation.KL)
        );
    }
}//NEEDS TEST