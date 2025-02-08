package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  LOW(0),
  MANUAL(0.585), 
  HORIZONTAL(0),
  FOURTYFIVE(0.625), // 0.569
  VERTICAL(0.608), // maybe?
  BACKOF(0.69),
  CLIMB(0);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
