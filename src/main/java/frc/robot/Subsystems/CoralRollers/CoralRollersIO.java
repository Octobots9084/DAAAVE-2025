package frc.robot.Subsystems.CoralRollers;

import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  @AutoLog
  public static class CoralRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hasCoral = false;
    public boolean coralInShute = false;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {
    hasCoral();
  }

  public default void setVoltage(double Position) {}

  public default boolean hasCoral() {
    return false;
  }

  public default boolean isIntaking() {
    return false;
  }

  public default void updateSim() {}
  ;
}
