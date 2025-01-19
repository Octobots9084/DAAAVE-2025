package frc.robot.Subsystems.CoralRollers;

public enum CoralRollersState {
  INTAKING(10),
  STOPPED(0),
  REJECTING(-10);

  public double voltage;

  CoralRollersState(double voltage) {
    this.voltage = voltage;
  }
}
