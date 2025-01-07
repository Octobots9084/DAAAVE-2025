package frc.robot.Subsystems.AlgaeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollersIO {
    @AutoLog
    public static class AlgaeRollersIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(AlgaeRollersIOInputs inputs) {}

    public default void setVoltage(double Position) {}

    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {}
}
