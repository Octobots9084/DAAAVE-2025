package frc.robot.Commands.complex;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.SetTargetReefSide;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.WristStates;

public class ClearAlgae extends SequentialCommandGroup {
    public ClearAlgae() {
        addCommands(
            new SetWristStateTolerance(WristStates.PREP, 0.05, ClosedLoopSlot.kSlot0),
            new SetDriveState(DriveState.AlignReef),
            
            new ConditionalCommand(
                new SetElevatorStateTolerance(ElevatorStates.TOPALGAE, 1.5).withTimeout(5),
                new SetElevatorStateTolerance(ElevatorStates.BOTTOMALGAE, 1.5).withTimeout(5),
                () -> {
                    ReefTargetOrientation targetOrientation = Swerve.getInstance().getReefTargetOrientation();
                    return (targetOrientation == ReefTargetOrientation.AB || targetOrientation == ReefTargetOrientation.EF || targetOrientation == ReefTargetOrientation.IJ); 
                }
            ),

            new SetWristStateTolerance(WristStates.ALGAEREMOVAL, 0.05, ClosedLoopSlot.kSlot0),
                
            new SetTargetReefSide(ReefTargetSide.ALGAE),
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(CoralRollersState.AlGAEINTAKING);
            }),
            new AlignReef(),
            new WaitForAlgae().withTimeout(3),
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.Reverse);
            }),
            new WaitCommand(0.3),
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.Manual);
            }));
    }
}


