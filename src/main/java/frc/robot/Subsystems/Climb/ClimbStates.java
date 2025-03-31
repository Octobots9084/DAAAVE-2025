package frc.robot.Subsystems.Climb;

public enum ClimbStates {
    // TODO change position values
    BreakFree(0),
    Deployed(0.587),
    Climbing(0.8795),
    Stored(0.08);

    public double position;

    ClimbStates(double position) {
        this.position = position;
    }

}
