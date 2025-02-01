package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class SetWristState extends InstantCommand {
  private WristStates targetState;
  private ClosedLoopSlot slot;

  public SetWristState(WristStates targetState, ClosedLoopSlot slot) {
    this.targetState = targetState;
    this.slot = slot;
  }

  @Override
  public void initialize() {
    Wrist.getInstance().setState(targetState, slot);
  }
}
