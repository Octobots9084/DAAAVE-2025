package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class EjectCoral extends SequentialCommandGroup {
    public EjectCoral() {
        BooleanSupplier wristAtL1 = () -> (Wrist.getInstance()
                .isAtState(WristStates.L1, 0.1));
        addCommands(
                new ConditionalCommand(new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.LEVEL1);
                }), 
                new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
                }), wristAtL1),

                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                }));
    }
}
