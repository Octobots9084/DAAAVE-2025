package frc.robot.Subsystems.Wrist;

public enum WristStates {
    // TODO edit these to the actual positions
    INTAKE(0.435),
    ELEPHANTIASIS(0.55),
    L1(0.587),
    L2(0.92),
    L3(0.91),
    L4(0.820),
    PREP(0.97),
    MANUAL(0.47),
    ALAGEREMOVAL(0.757),
    ALGAEPOP(0.757),
    ALAGESTACKREMOVAL(0.7),
    TUNING(0.7561);// TODO actually set this value
    // TODO actually set this value

    public double wristPosition;

    WristStates(double totalPosition) {
        this.wristPosition = totalPosition;
    }
}
