package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  LOW(0.192),
  INTAKE(0.012),
  MANUAL(0.2),
  HORIZONTAL(0),
  L1(0.223),

  L2(0.515),
  L3(0.515),
  L4(0.515),
  VERTICAL(0.437), // maybe?
  BACKOF(0.2),
  CLIMB(0),
  PREP(0.64);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
