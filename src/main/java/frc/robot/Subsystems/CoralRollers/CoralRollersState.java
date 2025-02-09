package frc.robot.Subsystems.CoralRollers;

public enum CoralRollersState {
  INTAKING(2),
  STOPPED(0),
  OUTPUT(-10);

  public double voltage;

  CoralRollersState(double voltage) {
    this.voltage = voltage;
  }
}
