package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class SetCoralRollersStateTolerance extends InstantCommand {
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

}
