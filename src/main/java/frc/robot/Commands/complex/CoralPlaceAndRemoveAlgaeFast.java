package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class CoralPlaceAndRemoveAlgaeFast extends SequentialCommandGroup{
    public CoralPlaceAndRemoveAlgaeFast(){
        BooleanSupplier isTop =  () -> Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.AB || Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.EF || Swerve.getInstance().getReefTargetOrientation() == States.ReefTargetOrientation.IJ;
        addCommands(
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.AlignReef);
            }),
            new ScoreCoral(),
            new InstantCommand(() -> {
                AlignVision.setPoleSide(ReefTargetSide.ALGAE);
            }),
            new WaitUntilCommand(() -> AlignVision.getInstance().isAligned()),
            new SetDriveState(DriveState.AlignReef),
            new SetWristState(WristStates.ALAGESTACKREMOVAL,ClosedLoopSlot.kSlot0),
            new ConditionalCommand(new SetElevatorState(ElevatorStates.TOPALGAE),new SetElevatorState(ElevatorStates.BOTTOMALGAE),isTop),
            new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.Reverse);
            }),
            new SetWristState(WristStates.PREP,ClosedLoopSlot.kSlot0),
            new SetElevatorState(ElevatorStates.LOW)
        );
    }
}
