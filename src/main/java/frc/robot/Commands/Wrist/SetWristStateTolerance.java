package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class SetWristStateTolerance extends Command {
  private WristStates targetState;
  private double tolerance;
  private Wrist wrist = Wrist.getInstance();
  private ClosedLoopSlot slot;

  public SetWristStateTolerance(WristStates targetState, double tolerance, ClosedLoopSlot slot) {
    this.targetState = targetState;
    this.tolerance = tolerance;
    this.slot = slot;
  }

  @Override
  public void initialize() {
    wrist.setState(targetState, slot);
  }

  @Override
  public boolean isFinished() {
    return wrist.isAtState(targetState, tolerance);
  }
}
