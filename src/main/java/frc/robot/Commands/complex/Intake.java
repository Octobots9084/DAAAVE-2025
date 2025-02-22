package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class Intake extends SequentialCommandGroup {
    public Intake() {
        if (Elevator.getInstance().getPosition() > Elevator.BOT_CROSSBAR_POS){
            addCommands(
                    new SetWristStateTolerance(WristStates.PREP,
                            0.05,
                            ClosedLoopSlot.kSlot0));
        }
        addCommands(
            new SetElevatorStateTolerance(ElevatorStates.INTAKE, 1.5),
            new SetWristStateTolerance(WristStates.INTAKE, 0.05, ClosedLoopSlot.kSlot0),
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
            }),
            new WaitForCoralDetected(),
            new WaitCommand(0.1),
            new InstantCommand(() -> {
                CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
            }));
    }

}
