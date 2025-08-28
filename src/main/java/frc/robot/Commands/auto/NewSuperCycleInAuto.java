package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.auto.testing.TestAlignAnyInAuto;
import frc.robot.Commands.auto.testing.TestAlignInAuto;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class NewSuperCycleInAuto extends SequentialCommandGroup {
    ReefTargetOrientation targetOrientation;
    ElevatorStates targetLevel;
    ReefTargetSide targetSide;

    public NewSuperCycleInAuto (ElevatorStates targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
        addCommands(
            new TestAlignInAuto(targetLevel, targetSide, targetOrientation),
            new PlaceCoralInAuto(targetLevel, targetSide, targetOrientation).withTimeout(2),

            new DriveBack().withTimeout(0.2),
            // new WaitCommand(0.15),
            
            new TestAlignInAuto(targetLevel, ReefTargetSide.ALGAE, targetOrientation),

            new RemoveAlgaeInAutoInSuperCycle(targetOrientation, targetSide),
            new WaitCommand(0.34),
            new ParallelCommandGroup(
                new SetDriveState(DriveState.Reverse),
                new SetElevatorState(ElevatorStates.LOW),
                new SetWristState(WristStates.PREP,ClosedLoopSlot.kSlot0)
            )
        );
    }
}//NEEDS TEST   