package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class RemoveAlgaeInAutoInSuperCycle extends Command{
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;

    public RemoveAlgaeInAutoInSuperCycle(ReefTargetOrientation targetOrientation, ReefTargetSide targetSide) {
        this.targetOrientation = targetOrientation;
        this.targetSide = targetSide;
    }

    @Override
    public void initialize() {
        BooleanSupplier isTop =  () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;
        new InstantCommand(() -> {
            AlignVision.setPoleSide(targetSide);
        });
        new WaitCommand(0.15);
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
        new WaitUntilCommand(() -> AlignVision.getInstance().isAligned());
        new SetWristState(WristStates.ALAGESTACKREMOVAL,ClosedLoopSlot.kSlot0);
        new ConditionalCommand(new SetElevatorState(ElevatorStates.TOPALGAEFAST),new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST),isTop);
        new SetCoralRollersState(CoralRollersState.ALGAEINTAKING);
        new WaitCommand(0.34);
        new ParallelCommandGroup(
            new DriveBack(),
            new SetElevatorState(ElevatorStates.LOW),
            new SetWristState(WristStates.PREP,ClosedLoopSlot.kSlot0)
        );
    }

    @Override
    public boolean isFinished() {
        return Elevator.getInstance().isAtState(ElevatorStates.LOW, 1) && Wrist.getInstance().getState() == WristStates.PREP;
    }
}