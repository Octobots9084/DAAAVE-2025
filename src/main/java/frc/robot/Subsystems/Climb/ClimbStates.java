package frc.robot.Subsystems.Climb;

public enum ClimbStates {
  // TODO change position values
  Deployed(-575),
  Climbing(0),
  Stored(0);

  public double position;

  ClimbStates(double position) {
    this.position = position;
  }

}
