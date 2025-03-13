package frc.robot.Subsystems.Elevator;

public enum ElevatorStates {
    // TODO edit these to the actual positions
    LOW(0),
    LEVEL1(0),
    LEVEL2(3.5),
    LEVEL3(17.57),
    LEVEL4(48.45),
    INTAKE(0),
    // all in terms of "arbitrary encoder units"
    MANUAL(0),
    BOTTOMALGAE(6.929), // TODO: make it below algae bottom so we can go up from there and push off
    // algae
    TOPALGAE(22.143), // TOTO: make it the tall algae bottom
    TOPALGAEREMOVAL(5),
    CLIMB(10);
    // TODO: decide if when removing algae we go bottom -> L4/L3 etc or bottom ->
    // enum top

    public double position;

    ElevatorStates(double totalPosition) {
        this.position = totalPosition;
    }
}
