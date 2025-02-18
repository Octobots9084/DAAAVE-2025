package frc.robot.Subsystems.Lights;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {
        Animations animationState;
    }

    public default void setAnimation(Animations animations) {}

    public default void setAnimation(Animations[] animations) {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LightsIOInputs inputs) {}

    public default void updateSim() {}
}
