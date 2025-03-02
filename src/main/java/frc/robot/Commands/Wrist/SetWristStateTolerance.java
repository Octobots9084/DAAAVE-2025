package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;
import frc.robot.Subsystems.Vision.AlignVision;

public class SetWristStateTolerance extends Command {
  private WristStates targetState;
  private ElevatorStates targetElevatorState;
  private double tolerance;
  private Wrist wrist = Wrist.getInstance();
  private ClosedLoopSlot slot;

  public SetWristStateTolerance(WristStates targetState, double tolerance, ClosedLoopSlot slot) {
    this.targetState = targetState;
    this.tolerance = tolerance;
    this.slot = slot;
  }

  public SetWristStateTolerance(ElevatorStates elevatorState, double tolerance, ClosedLoopSlot slot) {
    this.targetElevatorState = elevatorState;
    this.tolerance = tolerance;
    this.slot = slot;
  }

  @Override
  public void initialize() {
    if (targetState == null) {
      wrist.setState(targetElevatorState, slot);

    } else {
        if (targetState != WristStates.INTAKE) {
            wrist.setState(targetState, slot);
        }
    }
  }

  public void execute () {
    // Only let the wrist come back when we clear the reef
    if((AlignVision.getInstance().getLeftLidarDistance() > 0.4 || AlignVision.getInstance().getRightLidarDistance() > 0.4) && targetState == WristStates.INTAKE) {
        wrist.setState(targetState, slot);
    }
  }

  @Override
  public boolean isFinished() {
    if (targetState == null) {
      return wrist.isAtState(targetElevatorState, tolerance);

    } else {
      return wrist.isAtState(targetState, tolerance);

    }
  }
}
