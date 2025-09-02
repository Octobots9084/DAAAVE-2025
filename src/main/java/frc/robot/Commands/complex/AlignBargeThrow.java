package frc.robot.Commands.complex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class AlignBargeThrow extends SequentialCommandGroup{
    public AlignBargeThrow(){
        addCommands(
            new ParallelCommandGroup(new RotateToAngle(0), new DriveAwayFromBargeUntilAbleToAlign()),
            new SetElevatorState(ElevatorStates.LEVEL4),
            new AlignBarge().withTimeout(5),//MAKE ALIGN DO SOMEHTING
            // new WaitCommand(0.7),
            new BargeThrow()
        );
    }
}

/*
    zero gyro//
    can i raise the elevator?
    raise elevator//

    align barge//
    
    shoot//

    safe
 */