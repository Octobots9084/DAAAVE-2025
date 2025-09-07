package frc.robot.Commands.auto;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;

public class AlignInAuto extends Command {
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;
    private Debouncer debouncer;

    public AlignInAuto(ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
    }

    @Override
    public void initialize() {
        debouncer = new Debouncer(0.05);
        manager.orientation = targetOrientation;
        manager.selectedReefSide = targetSide;
        // manager.level = targetLevel;

        // AlignVision.setPoleLevel(manager.level);
        AlignVision.setPoleSide(manager.selectedReefSide);
        AlignVision.setReefOrientation(manager.orientation);
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));

    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Mode.SIM) {
            return true;
        }
        return AlignVision.getInstance().isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        // CommandScheduler.getInstance()
        //         .schedule(new SetWristStateTolerance(WristStates.PREP, 0.01, ClosedLoopSlot.kSlot0));
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
}//WORKS