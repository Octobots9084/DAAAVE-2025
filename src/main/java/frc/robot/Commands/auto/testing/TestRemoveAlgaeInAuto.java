package frc.robot.Commands.auto.testing;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeBottom;
import frc.robot.Commands.auto.testing.Algae.TestRemoveAlgaeTop;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.WristStates;

public class TestRemoveAlgaeInAuto extends SequentialCommandGroup {
    public TestRemoveAlgaeInAuto(ReefTargetOrientation targetOrientation) {
        BooleanSupplier isTop = () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF
                || targetOrientation == States.ReefTargetOrientation.IJ;
                addCommands(
                    // new TestAlignAnyInAuto(ReefTargetSide.ALGAE),
                    new InstantCommand(() -> {           
                        AlignVision.setPoleLevel(ElevatorStates.BOTTOMALGAE);
                        AlignVision.setPoleSide(ReefTargetSide.ALGAE);
                        AlignVision.setReefOrientation(targetOrientation);
                    }),
                    new WaitUntilCommand(5
                    ),
            new ConditionalCommand(
                    new TestRemoveAlgaeTop(),
                    new TestRemoveAlgaeBottom(),
                    isTop),
            new WaitUntilCommand(() -> CoralRollers.getInstance().isStalled()).withTimeout(2),
            new SetElevatorState(ElevatorStates.LOW),
            new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0)
        );
    }
}//NEEDS TEST (remove bottom and top need test also)
//removes, needs to stop rolling