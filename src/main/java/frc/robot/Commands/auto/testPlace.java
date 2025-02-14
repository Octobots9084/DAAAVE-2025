package frc.robot.Commands.auto;

import java.nio.file.attribute.PosixFilePermission;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.PlaceCoral;
import frc.robot.Commands.complex.PrepReefPlacement;
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

public class testPlace extends Command {
    ReefTargetLevel targetLevel;
    ReefTargetSide targetSide;
    ReefTargetOrientation targetOrientation;

    public testPlace(ReefTargetLevel targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("PLACRE", "PLACE");
        AlignVision.setPoleLevel(targetLevel);
        AlignVision.setPoleSide(targetSide);
        AlignVision.setReefOrientation(targetOrientation);
        Elevator.getInstance().setTargetState(ElevatorStates.LEVEL4);
        CommandScheduler.getInstance().schedule(new PrepReefPlacement());
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));

    }

    @Override
    public boolean isFinished() {
        if (AlignVision.getInstance().isAligned() && Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 1.5)
                && Wrist.getInstance().isAtState(ElevatorStates.LEVEL4, 0.01)) {
            CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
        }
        return AlignVision.getInstance().isAligned() && !CoralRollers.getInstance().HasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(new Intake());
        CommandScheduler.getInstance()
                .schedule(new SetElevatorState(ElevatorStates.INTAKE));
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }

}
