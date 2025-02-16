package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class PlaceCoral extends SequentialCommandGroup {
    public PlaceCoral() {
        BooleanSupplier elevatorAtL1 = () -> Elevator.getInstance()
                .getPosition() > (ElevatorStates.LEVEL1.position + 1.0);
        addCommands(
                new ConditionalCommand(new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
                }), new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.LEVEL1);
                }), elevatorAtL1),

                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                }));
    }
}
