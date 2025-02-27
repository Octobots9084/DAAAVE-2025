package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.hardware.CANrange;
import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {

  @AutoLog
  public static class CoralRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hasCoral = false;
    public boolean isIntaking = false;
    public double coralMeasureDist = 0;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {
  }

  public default void setVoltage(double Position) {
  }

  // coral is in the claw
  public default boolean HasCoral() {
    return false;
  }

  // coral is in the intake funnel area (NOT the claw)
  public default boolean IsIntaking() {
    return false;
  }

  public default boolean clawFrontSensorTriggered() {
    return false;
  }

  public default boolean clawBackSensorTriggered() {
    return false;
  }

  public default void rotateBy(double movement) {}

  public default void updateSim() {
  }
  public default boolean isStalled(){return false;}
}
