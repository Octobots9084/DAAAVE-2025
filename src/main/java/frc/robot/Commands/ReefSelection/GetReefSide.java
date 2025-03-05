package frc.robot.Commands.ReefSelection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.ControlMap;
import frc.robot.States;
import frc.robot.Subsystems.Swerve.Swerve;

public class GetReefSide {
  public static final CommandJoystick Right = ControlMap.CO_DRIVER_RIGHT;
  private double[][] reefPoints = {{0, -0.93}, {0.74, -0.39}, {0.74, 0.35}, {0, 0.99}, {-0.7, 0.36}, {-0.71, -0.4}};

  @SuppressWarnings("static-access")
  public States.ReefTargetOrientation joystickPos() {
    double xAxis = Right.getRawAxis(0);
    double yAxis = -Right.getRawAxis(1);

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

    xAxis = MathUtil.applyDeadband(xAxis, 0.075);
    yAxis = MathUtil.applyDeadband(yAxis, 0.075);
    if (xAxis == 0 && yAxis == 0) {
      return States.ReefTargetOrientation.NONE;
    }
    return Swerve.getInstance().getReefTargetOrientation().values()[sidePos];

  }
}
