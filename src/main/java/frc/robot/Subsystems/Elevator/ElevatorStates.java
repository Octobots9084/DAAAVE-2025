package frc.robot.Subsystems.Elevator;

public enum ElevatorStates {
  // TODO edit these to the actual positions
  LOW(0),
  LEVEL1(0),
  LEVEL2(0),
  LEVEL3(100),
  LEVEL4(3),
  INTAKE(0),
  CLIMB(0);

  public double position;

  ElevatorStates(double totalPosition) {
    this.position = totalPosition;
  }
}
