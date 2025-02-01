package frc.robot.Subsystems.Elevator;

public enum ElevatorStates {
  // TODO edit these to the actual positions
  LOW(0), // all in terms of "arbitrary encoder units"
  LEVEL1(0),
  LEVEL2(11),
  LEVEL3(27),
  LEVEL4(40),
  INTAKE(0);

  public double position;

  ElevatorStates(double totalPosition) {
    this.position = totalPosition;
  }
}
