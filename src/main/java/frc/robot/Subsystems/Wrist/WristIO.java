package frc.robot.Subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double wristPositionRotations = 0.0;
    public double wristVelocityRPM = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
    public double wristTemperature = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setState(WristStates state) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
