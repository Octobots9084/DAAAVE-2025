package frc.robot.Commands.Climb

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class DeployClimb extends InstantCommand {

  public DeployClimb() {}

  @Override
  public void initialize() {
    Climb.getInstance().setState(ClimbStates.Stored);
  }

}
