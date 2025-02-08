package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.hardware.CANrange;
import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  @AutoLog
  public static class CoralRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {
  }

  public default void setVoltage(double Position) {
  }

  public default boolean clawFrontSensorTriggered() {
    return false;
  }

  public default boolean clawBackSensorTriggered() {
    return false;
  }

  public default boolean chuteSensorTriggered() {
    return false;
  }

  public default void updateSim() {
  };
}
