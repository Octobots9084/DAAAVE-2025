package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  LOW(0),
  ANGLE1(0),
  ANGLE2(0),
  ANGLE3(0),
  CLIMB(0);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
