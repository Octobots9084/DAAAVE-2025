package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShowSelection {
  public static void displayReefSelection() {
    int reefState = manager.joystick.joystickPos().ordinal();
    if (reefState == 6) {
      manager.clearReef();
      manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);
    } else {
      manager.clearReef();
      manager.setReef(reefState, 0, true);
      manager.setReef(reefState, 1, true);
      manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);
    }
    for (int i = 0; i < manager.getReef().length; i++) {
      for (int j = 0; j < manager.getReef()[0].length; j++) {
        SmartDashboard.putBoolean(i * 2 + j + "", manager.getReefPos(i, j));
      }
    }

    for (int i = 0; i < manager.getLevels().length; i++) {
      SmartDashboard.putBoolean("L" + (i + 1), manager.getLevelPos(i));
    }
  }
}
