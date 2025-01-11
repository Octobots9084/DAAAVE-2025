package frc.robot.Subsystems.CoralRollers;

import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  @AutoLog
  public static class CoralRollersIOInputs {
    public double VelocityRPM = 0.0;
    public double AppliedVolts = 0.0;
    public double CurrentAmps = 0.0;
    public boolean mouthBeam = false;
    public boolean rearBeam = false;
  }

  public default void updateInputs(CoralRollersIOInputs inputs) {}

  public default void setVoltage(double Position) {}
}
