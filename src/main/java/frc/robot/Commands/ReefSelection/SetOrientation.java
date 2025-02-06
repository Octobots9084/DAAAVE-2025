package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States;
import frc.robot.Subsystems.Swerve.Swerve;

public class SetOrientation extends InstantCommand {
  private int side;
  private Swerve swerve = Swerve.getInstance();

  public SetOrientation(int side) {
    this.side = side;
  }

  @Override
  public void initialize() {
    States.ReefTargetOrientation reefState = swerve.getReefTargetOrientation();
    int reefStatePos = reefState.ordinal();
    if (reefState != States.ReefTargetOrientation.NONE) {
      manager.LastButtonPos[0] = reefStatePos;
      // replaces the terrible switching at last moment code
      if (manager.LastButtonPos[0] > 2) {
        manager.LastButtonPos[1] = (side == 0 ? 1 : 0);
      } else {
        manager.LastButtonPos[1] = side;
      }
      manager.clearReef();
      manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);

      swerve.setReefTargetSide(States.ReefTargetSide.values()[manager.LastButtonPos[1]]);
      swerve.setReefTargetOrientation(States.ReefTargetOrientation.values()[manager.LastButtonPos[0]]);
    }
  }
}
