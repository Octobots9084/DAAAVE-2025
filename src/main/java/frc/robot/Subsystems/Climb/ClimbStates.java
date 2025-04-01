package frc.robot.Subsystems.Climb;

public enum ClimbStates {
    // TODO change position values
    BreakFree(0),
    Deployed(0.741),
    Climbing(0.049),
    Stored(0.08);

    public double position;

    ClimbStates(double position) {
        this.position = position;
    }

}