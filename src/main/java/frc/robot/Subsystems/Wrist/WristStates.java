package frc.robot.Subsystems.Wrist;

public enum WristStates {
    // TODO edit these to the actual positions
    INTAKE(0.435),
    ELEPHANTIASIS(0.45),
    L1(0.587),
    L2(0.885),
    L3(0.885),
    L4(0.786),
    PREP(0.985),
    MANUAL(0.5),
    ALAGEREMOVAL(0.757),
    ALGAEPOP(0.757),
    TUNING(0.7561);// TODO actually set this value
    // TODO actually set this value

    public double wristPosition;

    WristStates(double totalPosition) {
        this.wristPosition = totalPosition;
    }
}
