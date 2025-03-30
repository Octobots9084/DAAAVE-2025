package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.auto.testAlgae;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class RemoveAlgaeInAuto extends SequentialCommandGroup {
    public RemoveAlgaeInAuto(ReefTargetOrientation targetOrientation) {
        BooleanSupplier isTop = () -> targetOrientation == States.ReefTargetOrientation.AB || targetOrientation == States.ReefTargetOrientation.EF || targetOrientation == States.ReefTargetOrientation.IJ;
        addCommands(
            new InstantCommand(() -> {
                Swerve.getInstance().driveRobotRelative(new ChassisSpeeds(-1.2, 0, 0));
            }),
            new WaitCommand(0.1),
            new ConditionalCommand(new SetElevatorState(ElevatorStates.TOPALGAE),new SetElevatorState(ElevatorStates.BOTTOMALGAE), isTop),
            new WaitCommand(0.1),
            new testAlgae(targetOrientation),
            new RemoveAlgaeBottom()
        );
    }
}
