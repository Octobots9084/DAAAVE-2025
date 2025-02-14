package frc.robot.Subsystems.Elevator;

public enum ElevatorStates {
  // TODO edit these to the actual positions
  LOW(0), // all in terms of "arbitrary encoder units"
  MANUAL(50),
  LEVEL1(0),
  LEVEL2(8),
  LEVEL3(55.8),
  LEVEL4(136),
  BOTTOMALGAE(0), // TODO: make it below algae bottom so we can go up from there and push off
                  // algae
  TOPALGAE(0), // TOTO: make it the tall algae bottom
  INTAKE(0);// TODO: decide if when removing algae we go bottom -> L4/L3 etc or bottom ->
            // enum top

  public double position;

  ElevatorStates(double totalPosition) {
    this.position = totalPosition;
  }
}
