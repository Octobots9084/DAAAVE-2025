package frc.robot.Commands.ReefSelection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.ControlMap;

public class GetReefSide {
  public static final CommandJoystick CO_DRIVER_LEFT = ControlMap.CO_DRIVER_LEFT;
  private int[][] reefPoints = {{-1, 0}, {0, 1}, {1, 1}, {1, -1}, {0, -1}, {-1, -1}};

  public manager.joystickState joystickPos() {
    double xAxis = CO_DRIVER_LEFT.getRawAxis(0);
    double yAxis = -CO_DRIVER_LEFT.getRawAxis(1);

    double minDist = 5;
    int sidePos = 0;
    double currentDist;

    for (int i = 0; i < reefPoints.length; i++) {
      currentDist =
          Math.sqrt(
              Math.sqrt(Math.abs(reefPoints[i][0] - xAxis) + Math.abs(reefPoints[i][1] - yAxis)));
      if (currentDist < minDist) {
        minDist = currentDist;
        sidePos = i;
      }
    }

    xAxis = MathUtil.applyDeadband(xAxis, 0.05);
    yAxis = MathUtil.applyDeadband(yAxis, 0.05);
    if (xAxis == 0 && yAxis == 0) {
      return manager.joystickState.NONE;
    }
    return manager.joystickState.values()[sidePos];
  }
}
