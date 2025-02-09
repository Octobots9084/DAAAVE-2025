package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  LOW( 0.192),
  MANUAL(0.2), 
  HORIZONTAL(0),
  FOURTYFIVE(0.515),
  VERTICAL(0.437), // maybe?
  BACKOF(0.2),
  CLIMB(0);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
