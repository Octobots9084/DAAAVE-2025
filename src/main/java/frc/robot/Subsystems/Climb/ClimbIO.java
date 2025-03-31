package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double positionRotations;
        public double velocityRPM;
        public double appliedVolts;
        public double busVoltage;
        public double currentAmps;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setPosition(double newPosition, ClosedLoopSlot slot) {}

    public default double getPosition() {
        return 0;
    }

    public default void allStop() {}

    public default void updateSim() {}

    public default void setVoltage(double voltage) {};

    public default void setTalonVoltage(double voltage) {};

    public default boolean talonIsStalled() {
        return false;
    }
}