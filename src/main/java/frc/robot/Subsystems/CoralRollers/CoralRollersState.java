package frc.robot.Subsystems.CoralRollers;

public enum CoralRollersState {
    INTAKING(-4.7),
    STOPPED(0),
    OUTPUT(10),
    LEVEL1(-10),
    ALGAEINTAKING(-10),
    AlGAEOUTPUT(10);

    public double voltage;

    CoralRollersState(double voltage) {
        this.voltage = voltage;
    }
}
