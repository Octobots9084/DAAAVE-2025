package frc.robot.Commands.auto;

import java.io.SequenceInputStream;
import java.util.function.BooleanSupplier;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class RemoveAlgaeInAutoInSuperCycle extends SequentialCommandGroup{
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;

    public RemoveAlgaeInAutoInSuperCycle(ReefTargetOrientation targetOrientation, ReefTargetSide targetSide) {
        this.targetOrientation = targetOrientation;
        this.targetSide = targetSide;
        BooleanSupplier isTop =  () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;

        addCommands(
            new ParallelCommandGroup(
                new AlignInAuto(ReefTargetSide.ALGAE, targetOrientation),
                new SetWristStateTolerance(WristStates.ALAGESTACKREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
                // new ParallelCommandGroup(
                new ConditionalCommand(
                    new SetElevatorState(ElevatorStates.TOPALGAEFAST),
                    new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),
                    isTop)
            ),
            // ),
            new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            new WaitUntilCommand(() -> CoralRollers.getInstance().isStalled()),
            new DriveBack().withTimeout(0.3).withTimeout(1)
        );
    }

    // @Override
    // public void initialize() {
        // new InstantCommand(() -> {
        //     AlignVision.setPoleSide(targetSide);
        // });
        // new WaitCommand(0.15);
        // Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
        // new WaitUntilCommand(() -> AlignVision.getInstance().isAligned());
        
    // }

    // @Override
    // public boolean isFinished() {
    //     return Elevator.getInstance().isAtState(ElevatorStates.LOW, 1) && Wrist.getInstance().getState() == WristStates.PREP;
    // }
}