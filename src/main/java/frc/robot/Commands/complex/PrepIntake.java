package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.ReefSelection.SetTargetReefLevel;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepIntake extends Command {

    private boolean isBack;
    private int timer;

    public PrepIntake(boolean beBack) {
        isBack = beBack;
        timer = 0;
    }

    @Override
    public void execute() {
        if (!isBack) {
            isBack = (timer >= 20);// TODO Change 20 to actcull timer
            Swerve swerveInstance = Swerve.getInstance();// TODO get io
            Swerve.getInstance().driveRobotRelative(new ChassisSpeeds(
                    0,
                    0.1 * swerveInstance.getIo().getMaxSpeed(),
                    0));
        } else {
            if (Elevator.getInstance().getTargetState() != ElevatorStates.INTAKE) {
                CommandScheduler.getInstance()
                        .schedule(new SetTargetReefLevel(ElevatorStates.INTAKE));
            } else {
                if (Wrist.getInstance().getState() != WristStates.INTAKE
                        && Elevator.getInstance().getPosition() > 0.2) {// TODO set actcul cross bar postion and >/<
                    Wrist.getInstance().setState(WristStates.INTAKE, ClosedLoopSlot.kSlot0);// TODO change K slot to
                                                                                            // correct one
                }
            }

        }
    }

    @Override
    public boolean isFinished() {
        if (Wrist.getInstance().isAtState(WristStates.INTAKE, 0.01)
                && Elevator.getInstance().isAtState(ElevatorStates.INTAKE, 0.01)) {// TODO change tolerance x2 (wrist +
                                                                                   // elevator)
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Double currentWristpostions = Wrist.getInstance().getPosition();
            WristStates newWristPosition = WristStates.MANUAL;
            newWristPosition.wristPosition = currentWristpostions;
            Wrist.getInstance().setState(newWristPosition, ClosedLoopSlot.kSlot0);

            Double currentElevatorpostions = Elevator.getInstance().getPosition();
            ElevatorStates newElevatorPosition = ElevatorStates.MANUAL;
            newElevatorPosition.position = currentElevatorpostions;
            Elevator.getInstance().setState(newElevatorPosition);
        }
    }
}
