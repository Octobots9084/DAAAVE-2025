package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class BargeThrow extends SequentialCommandGroup{
    public BargeThrow(){
        BooleanSupplier inPrep = () -> Wrist.getInstance().getState() == WristStates.PREP;
        SmartDashboard.putBoolean("Barge Throw In Prep", inPrep.getAsBoolean());
        addCommands(
            // new SetCoralRollersState(CoralRollersState.ALGAEINTAKING),
                new SetWristStateTolerance(WristStates.BARGEALGAE, 0.04, ClosedLoopSlot.kSlot0),


            new SetElevatorStateTolerance(ElevatorStates.LEVEL4, 1),      
            // new WaitCommand(0.7),
            // new SetWristState(WristStates.BARGEALGAE, ClosedLoopSlot.kSlot0),
            // new WaitCommand(0.4),
            new WaitUntilCommand(()->
                Wrist.getInstance().isAtState(WristStates.BARGEALGAE, 0.05)
            ),
            new SetCoralRollersState(CoralRollersState.AlGAEOUTPUT),
            // new WaitCommand(0.25),
            new SetElevatorStateTolerance(ElevatorStates.LOW, 10).withTimeout(3),

            // new WaitCommand(0.5),
            // new SetCoralRollersState(CoralRollersState.STOPPED).withTimeout(1)
            new SetCoralRollersState(CoralRollersState.STOPPED)

        );
    }
}
