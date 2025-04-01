package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class ScoreCoralAndBackOff extends SequentialCommandGroup {
    public ScoreCoralAndBackOff() {
        addCommands(
                new ScoreCoral(),
                new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Reverse);
                }),
                new ParallelCommandGroup(new WaitCommand(.25).andThen(new InstantCommand(() -> {
                    CommandScheduler.getInstance().schedule(new Intake());
                })), new WaitCommand(0.5).andThen(new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Manual);
                }))));
    }
}
