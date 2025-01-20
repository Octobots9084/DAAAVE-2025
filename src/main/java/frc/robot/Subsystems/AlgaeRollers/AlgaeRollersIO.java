package frc.robot.Subsystems.AlgaeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollersIO {
  @AutoLog
  public static class AlgaeRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean beamValue = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AlgaeRollersIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default double getVoltage() {
    return 0;
  }

  public default void updateSim() {}

  public default boolean hasAlgae() {
    return false;
  }
}
