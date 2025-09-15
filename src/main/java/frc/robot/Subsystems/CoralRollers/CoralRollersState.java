package frc.robot.Subsystems.CoralRollers;

import frc.robot.Commands.Emote.BrazilianCycle;

public enum CoralRollersState {
    INTAKING(-4.7),
    STOPPED(0),
    OUTPUT(10),
    LEVEL1(-6.75),
    ALGAEINTAKING(-13),
    AlGAEOUTPUT(10),
    ALGAEHOLD(-7), //TODO update this value based on testing

    BrazilianCycle(-12);

    public double voltage;

    CoralRollersState(double voltage) {
        this.voltage = voltage;
    }
}
