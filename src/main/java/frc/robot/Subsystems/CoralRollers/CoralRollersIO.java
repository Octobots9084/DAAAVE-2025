package frc.robot.Subsystems.CoralRollers;

import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  @AutoLog
  public static class CoralRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean isIntaking = false;
    public boolean hasCoral = false;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {}

  public default void setVoltage(double Position) {}

  public default double getVoltage() {
    return 0;
  }

  public default void updateSim() {}

  public default boolean hasCoral() {
    return false;
  }
}
