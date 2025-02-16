package frc.robot.Subsystems.Elevator;

public enum ElevatorStates {
    // TODO edit these to the actual positions
    LOW(0.1 / 3),
    LEVEL1(0.5 / 3),
    LEVEL2(8 / 3.0),
    LEVEL3(55.8 / 3.0),
    LEVEL4(145 / 3.0),
    INTAKE(0),
    // all in terms of "arbitrary encoder units"
    MANUAL(50 / 3.0),
    BOTTOMALGAE(0), // TODO: make it below algae bottom so we can go up from there and push off
                    // algae
    TOPALGAE(0); // TOTO: make it the tall algae bottom
    // TODO: decide if when removing algae we go bottom -> L4/L3 etc or bottom ->
    // enum top

    public double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }
}
