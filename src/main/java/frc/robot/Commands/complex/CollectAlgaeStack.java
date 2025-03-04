package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Wrist.WristStates;

public class CollectAlgaeStack extends SequentialCommandGroup{
    public CollectAlgaeStack(){
        addCommands(
            new SetWristState(WristStates.ALAGESTACKREMOVAL, ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.OUTPUT)
        );
    }
}
