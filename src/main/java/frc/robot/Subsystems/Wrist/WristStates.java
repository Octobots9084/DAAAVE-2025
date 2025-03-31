package frc.robot.Subsystems.Wrist;

public enum WristStates {
    // TODO edit these to the actual positions
    INTAKE(0.375),
    ELEPHANTIASIS(0.425),
    L1(0.547),
    L2(0.828),
    L3(0.828),
    L4(0.745),
    PREP(0.92),
    MANUAL(0.45),
    ALGAEREMOVAL(0.678),
    TOPALGAEREMOVAL(0.678),

    ALAGESTACKREMOVAL(0.610),
    BARGEALGAE(0.83),
    TUNING(0.7561), // TODO actually set this value
    TOPALGAEINTAKE(00000000),
    BOTTOMALGAEINTAKE(00000000),
    QUICKALGAEREMOVALLOW(0.575),
    QUICKALGAEREMOVALHIGH(0.575),
    QUICKALGAERCOLLECTION(0.575);

    public double wristPosition;

    WristStates(double totalPosition) {
        this.wristPosition = totalPosition;
    }
}
