package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class SetCoralRollersState extends Command {
  private CoralRollersState targetState;

  public SetCoralRollersState(CoralRollersState targetState) {
    this.targetState = targetState;
  }

  @Override
  public void initialize() {
    CoralRollers.getInstance().setState(targetState);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
