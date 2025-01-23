package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.*;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.WristStates;

public class AlignReef extends Command {
  private ElevatorStates elevatorState;
  private WristStates wristState;
  private Swerve swerve = Swerve.getInstance();
  private ReefAlignmentPosition alignmentPosition;
  private boolean aligningRight;

  public AlignReef(ReefTargetHeight level, ReefAlignmentPosition alignmentPosition) {
    this.alignmentPosition = alignmentPosition;
    if (level == ReefTargetHeight.L1) {
      elevatorState = ElevatorStates.LEVEL1;
      wristState = WristStates.HORIZONTAL;
    } else if (level == ReefTargetHeight.L2) {
      elevatorState = ElevatorStates.LEVEL2;
      wristState = WristStates.FOURTYFIVE;
    } else if (level == ReefTargetHeight.L3) {
      elevatorState = ElevatorStates.LEVEL3;
      wristState = WristStates.FOURTYFIVE;
    } else {
      elevatorState = ElevatorStates.LEVEL4;
      wristState = WristStates.VERTICAL;
    }
  }

  @Override
  public void initialize() {
    if (alignmentPosition == ReefAlignmentPosition.RIGHT) {
      swerve.setDriveState(DriveState.AlignReefRight);
      aligningRight = true;
    } else {
      swerve.setDriveState(DriveState.AlignReefLeft);
      aligningRight = false;
    }

    // TODO
    // CommandScheduler.getInstance().schedule(new PrepReefPlacement(elevatorState, wristState));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setDriveState(DriveState.Manual);
  }

  // @Override
  // public boolean isFinished() {
  //   if ((aligningRight && swerve.isAlignedToCoralRight)
  //       || (!aligningRight && swerve.isAlignedToCoralLeft)) {
  //     CommandScheduler.getInstance()
  //         .schedule(new SetCoralRollersState(CoralRollersState.REJECTING));
  //   }
  //   // check if the coral has left robot
  //   // if true return true
  //   return false;
  // }
}
