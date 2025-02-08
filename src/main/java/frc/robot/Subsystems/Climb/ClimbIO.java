package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double encoderPosition;
        public double appliedVolts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs) {}


    public default void setPosition(double newPosition) {}

    public default double getPosition() {
        return 0;
    }

    public default void updateSim() {}

    
}