package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class SetAlgaeRollerState extends InstantCommand {
  private CoralRollersState targetState;

  //if you are questioning in the future why this is here and not just use the coral roller states this is for organizational purposes/superiority
  public SetAlgaeRollerState(CoralRollersState targetState) {
    this.targetState = targetState;
  }

  @Override
  public void initialize() {
    CoralRollers.getInstance().setState(targetState);
  }

}
