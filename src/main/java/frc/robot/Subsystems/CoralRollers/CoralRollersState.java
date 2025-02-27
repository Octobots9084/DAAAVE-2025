package frc.robot.Subsystems.CoralRollers;

public enum CoralRollersState {
    INTAKING(-4),
    STOPPED(0),
    OUTPUT(10),
    LEVEL1(-10);

    public double voltage;

    CoralRollersState(double voltage) {
        this.voltage = voltage;
    }
}
