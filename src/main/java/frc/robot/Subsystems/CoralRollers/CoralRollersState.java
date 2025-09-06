package frc.robot.Subsystems.CoralRollers;

public enum CoralRollersState {
    INTAKING(-4.7),
    STOPPED(0),
    OUTPUT(10),
    LEVEL1(-6.75),
    ALGAEINTAKING(-10),
    AlGAEOUTPUT(10),
    ALGAEHOLD(-7); //TODO update this value based on testing

    public double voltage;

    CoralRollersState(double voltage) {
        this.voltage = voltage;
    }
}
