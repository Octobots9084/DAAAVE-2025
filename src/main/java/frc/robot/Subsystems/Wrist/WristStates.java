package frc.robot.Subsystems.Wrist;

public enum WristStates {
  // TODO edit these to the actual positions
  INTAKE(0.012),
  L1(0.27),
  L2(0.55),
  L3(0.55),
  L4(0.475),
  PREP(0.64),
  MANUAL(0.55);

  public double wristPosition;

  WristStates(double totalPosition) {
    this.wristPosition = totalPosition;
  }
}
