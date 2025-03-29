package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.WristStates;

public class CoralPlaceAndRemoveAlgaeFast extends SequentialCommandGroup{
    public CoralPlaceAndRemoveAlgaeFast(){
        Swerve swerve = Swerve.getInstance();
        BooleanSupplier isHighAlgae = () -> swerve.getReefTargetOrientation() == States.ReefTargetOrientation.AB || swerve.getReefTargetOrientation() == States.ReefTargetOrientation.EF || swerve.getReefTargetOrientation() == States.ReefTargetOrientation.IJ;
        addCommands(
            new InstantCommand(() -> {
                AlignVision.setReefOrientation(swerve.getReefTargetOrientation());
            }),
            new InstantCommand(() -> {
                AlignVision.setPoleSide(swerve.getReefTargetSide());
            }),
            new InstantCommand(() -> {
                swerve.setDriveState(DriveState.AlignReef);
            }),
            new ScoreCoral(),
            new InstantCommand(() -> {
                AlignVision.setPoleSide(States.ReefTargetSide.ALGAE);
            }),
            new InstantCommand(() -> {
                swerve.setDriveState(DriveState.AlignReef);
            }),
            new WaitCommand(0.2),
            new WaitUntilCommand(() -> AlignVision.getInstance().isAligned()),
            // new SetWristState(WristStates.QUICKALGAEREMOVAL, ClosedLoopSlot.kSlot0),
            new ConditionalCommand(new SetElevatorState(ElevatorStates.TOPALGAEFAST), new SetElevatorState(ElevatorStates.BOTTOMALGAEFAST), isHighAlgae),
            new InstantCommand(() -> {
                swerve.setDriveState(DriveState.Reverse);
            }),
            new WaitCommand(0.1),
            new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),
            new InstantCommand(() -> {
                swerve.setDriveState(DriveState.Manual);
            })
        );
    }
}
