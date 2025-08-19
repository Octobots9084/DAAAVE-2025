package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.WristStates;

public class BargeThrow extends SequentialCommandGroup{
    public BargeThrow(){
        addCommands(
            new RotateToAngle(0),       
            new DriveAwayFromBargeUntilAbleToAlign(),
            new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0),
            new SetElevatorState(ElevatorStates.LEVEL4),
            new AlignBarge().withTimeout(5),//MAKE ALIGN DO SOMEHTING
            // new WaitCommand(0.7),
            new SetWristState(WristStates.BARGEALGAE,ClosedLoopSlot.kSlot0),
            new SetCoralRollersState(CoralRollersState.AlGAEOUTPUT),

            // new WaitCommand(0.1),
            new WaitUntilCommand(() -> !CoralRollers.getInstance().isStalled()),//maybe works? test. if not, do the wait ^^

            new RobotSafeState()
        );
    }
}

/*
    zero gyro//

    raise elevator//

    align barge//
    
    shoot//

    safe
 */