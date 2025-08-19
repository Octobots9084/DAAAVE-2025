package frc.robot.Commands.auto.testing;

import java.nio.file.attribute.PosixFilePermission;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.auto.PrepReefPlacementAuto;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.EjectCoral;
import frc.robot.Commands.complex.PrepReefPlacement;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class TestPlaceCoralInAuto extends Command {
    private Debouncer debouncer;

    @Override
    public void initialize() {
        debouncer = new Debouncer(0.05);

        manager.level = ElevatorStates.LEVEL4;
        CommandScheduler.getInstance().schedule(new PrepReefPlacementAuto());
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Mode.SIM) {
            return true;
        }
        if (Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 1.5)
                && Wrist.getInstance().isAtState(ElevatorStates.LEVEL4, 0.01)) {
            CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
        }
        return !CoralRollers.getInstance().HasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance()
                .schedule(new SetWristStateTolerance(WristStates.PREP, 0.01, ClosedLoopSlot.kSlot0));
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
        CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
    }
}//WORKS