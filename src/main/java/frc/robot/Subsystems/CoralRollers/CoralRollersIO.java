package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.hardware.CANrange;
import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  CANrange coralDetector = new CANrange(0);

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
    return coralDetector.getDistance().getValueAsDouble() < 0.5;
  }

  public default boolean coralInShute() {
    return false;
  }

  public default void updateSim() {}
  ;
}
