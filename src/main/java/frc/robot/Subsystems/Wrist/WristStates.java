package frc.robot.Subsystems.Wrist;

public enum WristStates {
    // TODO edit these to the actual positions
    INTAKE(0.375),
    ELEPHANTIASIS(0.425),
    L1(0.5),
    L2(0.864),
    L3(0.864),
    L4(0.745),
    PREP(0.92),
    MANUAL(0.45),
    ALGAEREMOVAL(0.678),
    TOPALGAEREMOVAL(0.678),

    ALAGESTACKREMOVAL(0.610),
    BARGEALGAE(0.9),//TODO:make sure this is good
    TUNING(0.7561), // TODO actually set this value
    TOPALGAEINTAKE(00000000),
    BOTTOMALGAEINTAKE(00000000),
    QUICKALGAEREMOVALLOW(0.575),
    QUICKALGAEREMOVALHIGH(0.575),
    GROUNDALGAE(0.56),

    
    BrazilianCycle(0.78),
    BrazilianCycle2(0.72),
    BrazilianCycle3(0.745),
    BOTTOMALGAE2(0.83);

    public double wristPosition;

    WristStates(double totalPosition) {
        this.wristPosition = totalPosition;
    }
}
