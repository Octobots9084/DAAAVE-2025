package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class SetCoralRollersStateTolerance extends Command {
  private CoralRollersState targetState;
  // private double tolerance;
  private CoralRollers coralRollers = CoralRollers.getInstance();

  public SetCoralRollersStateTolerance(CoralRollersState targetState, double tolerance) {
    this.targetState = targetState;
    // this.tolerance = tolerance;
  }

  @Override
  public void initialize() {
    coralRollers.setState(targetState);
  }

  @Override
  public boolean isFinished() {
    return true;
    // return coralRollers.isAtState(targetState,tolerance);
  }
}
