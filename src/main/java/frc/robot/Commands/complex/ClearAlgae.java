package frc.robot.Commands.complex;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.SetTargetReefSide;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.WristStates;

public class ClearAlgae extends SequentialCommandGroup {
    public ClearAlgae() {
        addCommands(
            new ConditionalCommand(
                new SetWristStateTolerance(WristStates.PREP,
                0.05, ClosedLoopSlot.kSlot0),
                new InstantCommand(),
                () -> { return Elevator.getInstance().getPosition() <= Elevator.BOT_CROSSBAR_POS; }),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new SetElevatorStateTolerance(ElevatorStates.TOPALGAE, 1.5),
                    new SetElevatorStateTolerance(ElevatorStates.BOTTOMALGAE, 1.5),
                    () -> { 
                        ReefTargetOrientation targetOrientation = Swerve.getInstance().getReefTargetOrientation();
                        return (targetOrientation == ReefTargetOrientation.AB || targetOrientation == ReefTargetOrientation.EF || targetOrientation == ReefTargetOrientation.IJ); 
                    }),
                new SetWristStateTolerance(WristStates.ALAGEREMOVAL,
                0.05, ClosedLoopSlot.kSlot0)
            ),
                
            new SetTargetReefSide(ReefTargetSide.ALGAE),
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
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


