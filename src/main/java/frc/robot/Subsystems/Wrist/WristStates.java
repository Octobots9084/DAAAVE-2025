package frc.robot.Subsystems.Wrist;

public enum WristStates {
    // TODO edit these to the actual positions
    INTAKE(0.435),
    L1(0.587),
    L2(0.866),
    L3(0.866),
    L4(0.828),
    PREP(0.97),
    MANUAL(0.554),
    ALAGEREMOVAL(0),
    TUNING(0.7561);// TODO actually set this value
    // TODO actually set this value

    public double wristPosition;

    WristStates(double totalPosition) {
        this.wristPosition = totalPosition;
    }
}
