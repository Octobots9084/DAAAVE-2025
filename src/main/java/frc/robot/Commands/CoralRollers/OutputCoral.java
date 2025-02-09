package frc.robot.Commands.CoralRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class OutputCoral extends Command {
  @Override
  public void initialize() {
    CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return !CoralRollers.getInstance().HasCoral();
  }

  @Override
  public void end (boolean interrupted) {
    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
  }
}
