package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.Wrist;

public class ScoreCoral extends Command {
  public boolean elevatorInPos = false;
  public boolean wristInPosition = false;
  public boolean isAligned = false;
  private Swerve swerve = Swerve.getInstance();
  private CoralRollers coralRollers = CoralRollers.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private ElevatorStates targetElevatorState;
  private ReefTargetSide targetSide;
  private ReefTargetOrientation targetOrientation;

  public ScoreCoral(
      ElevatorStates elevatorState, ReefTargetSide side, ReefTargetOrientation orientation) {
    targetElevatorState = elevatorState;
    targetSide = side;
    targetOrientation = orientation;
  }

  @Override
  public void initialize() {
    if (coralRollers.io.hasCoral()) {
      swerve.setDriveState(DriveState.AlignReef);
      CommandScheduler.getInstance().schedule(new SetTargetReefLevel(targetElevatorState));
      CommandScheduler.getInstance().schedule(new SetTargetReefSide(targetSide));
      CommandScheduler.getInstance().schedule(new SetTargetReefOrientation(targetOrientation));
    }
  }

  @Override
  public void execute() {
    elevatorInPos = elevator.isAtState(targetElevatorState, 0.1 /*set actual tolerance*/);
    wristInPosition = wrist.isAtState(targetElevatorState, 0.1 /*set actual tolerance*/);
    isAligned = true; // Need vision code

    if (elevatorInPos
        && wristInPosition
        && isAligned
        && (coralRollers.getState() != CoralRollersState.REJECTING))
      CommandScheduler.getInstance()
          .schedule(new SetCoralRollersState(CoralRollersState.REJECTING));
    //
  }

  @Override
  public boolean isFinished() {
    if (coralRollers.io.hasCoral() == false) return true;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setDriveState(DriveState.Manual);
    if (!interrupted) // when coral has been scored send elevator back down after 0.1 seconds
    CommandScheduler.getInstance()
          .schedule(new WaitCommand(0.1).andThen(new SetElevatorState(ElevatorStates.INTAKE)));
  }
  // new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.REJECTING))
}
