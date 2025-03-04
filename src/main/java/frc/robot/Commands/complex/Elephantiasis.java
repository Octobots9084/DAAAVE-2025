package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.Wrist.WristStates;

public class Elephantiasis extends SequentialCommandGroup {
    public Elephantiasis() {
        addCommands(
            new SequentialCommandGroup(
                new SetWristStateTolerance(WristStates.ELEPHANTIASIS, 0.035, ClosedLoopSlot.kSlot0),
                new SetWristStateTolerance(WristStates.INTAKE, 0.035, ClosedLoopSlot.kSlot0)
            )
        );
    }
}
