package frc.robot.Subsystems.AlgaeRollers;

public enum AlgaeRollersStates {
    // TODO change volt values
    INTAKE(0),
    OUTPUT(0),
    OFF(0);

    public double voltage;

    AlgaeRollersStates(double volts) {
        this.voltage = volts;
    }
}
