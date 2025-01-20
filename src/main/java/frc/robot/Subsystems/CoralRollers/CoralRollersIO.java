package frc.robot.Subsystems.CoralRollers;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.CANrange;

public interface CoralRollersIO {
  public CANrange coralDetector = new CANrange(0);
  @AutoLog
  public static class CoralRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hasCoralFromSource = false;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {
    hasCoral();
  }

  public default void setVoltage(double Position) {}

  public default boolean hasCoral() {
    return coralDetector.getDistance().getValueAsDouble()<0.05;
  }

  public default boolean isIntaking() {
    return false;
  }

  public void updateSim();
}
