package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  LOW(0),
  HORIZONTAL(0),
  FOURTYFIVE(0.585), // 0.569
  VERTICAL(0.608), // maybe?
  BACKOF(0.74),
  CLIMB(0);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
