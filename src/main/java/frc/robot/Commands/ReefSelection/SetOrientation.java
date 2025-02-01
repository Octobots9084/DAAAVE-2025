package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetOrientation extends InstantCommand {
  private int side;

  public SetOrientation(int side) {
    this.side = side;
  }

  @Override
  public void initialize() {
    manager.joystickState reefState = manager.joystick.joystickPos();
    int reefStatePos = reefState.ordinal();
    if (reefState != manager.joystickState.NONE) {
      manager.LastButtonPos[0] = reefStatePos;
      // replaces the terrible switching at last moment code
      if (manager.LastButtonPos[0] > 2) {
        manager.LastButtonPos[1] = (side == 0 ? 1 : 0);
      } else {
        manager.LastButtonPos[1] = side;
      }
      manager.clearReef();
      manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);
    }
  }
}
