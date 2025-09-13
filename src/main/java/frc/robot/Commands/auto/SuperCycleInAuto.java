package frc.robot.Commands.auto;

import java.util.function.BooleanSupplier;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.auto.testing.TestAlignAnyInAuto;
import frc.robot.Commands.auto.testing.TestPlaceCoralInAuto;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Wrist.*;

public class SuperCycleInAuto extends SequentialCommandGroup {
    ReefTargetOrientation targetOrientation;
    ElevatorStates targetLevel;
    ReefTargetSide targetSide;

    public SuperCycleInAuto (ElevatorStates targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
        addCommands(
            // new AlignInAuto(targetSide, targetOrientation),
            new PlaceCoralInAuto(targetLevel, targetSide, targetOrientation).withTimeout(4),
            // new WaitCommand(0.15),
            // new AlignInAuto(ReefTargetSide.ALGAE, targetOrientation),//works before this
            // new AlignInAuto(ReefTargetSide.ALGAE, targetOrientation),//works before this
            // new SetElevatorState(targetLevel),

            // new DriveBack().withTimeout(0.5),//works abive this

            new RemoveAlgaeInAutoInSuperCycle(targetOrientation, ReefTargetSide.ALGAE)
            // new WaitCommand(0.34)
            // new DriveBack().withTimeout(0.2)//can get rid of this maybe
        );
    }
}