package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class LoadCoral extends Command {
  @Override
  public void initialize() {
    CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
  }

  @Override
  public boolean isFinished() {
    return CoralRollers.getInstance().hasCoral();
  }

  @Override
  public void end (boolean interrupted) {
    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
  }
}
