package frc.robot.Commands.auto.testing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.auto.AlignInAuto;
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
            new AlignInAuto(targetSide, ReefTargetOrientation.AB),
            new AlignInAuto(targetSide, ReefTargetOrientation.CD),
            new AlignInAuto(targetSide, ReefTargetOrientation.EF),
            new AlignInAuto(targetSide, ReefTargetOrientation.GH),
            new AlignInAuto(targetSide, ReefTargetOrientation.IJ),
            new AlignInAuto(targetSide, ReefTargetOrientation.KL)
        );
    }
}//NEEDS TEST