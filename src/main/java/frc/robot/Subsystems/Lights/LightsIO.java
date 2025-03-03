package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {
        TimedAnimation animationState;
    }

    public default void setAnimation(TimedAnimation animations) {}

    public default void setAnimation(TimedAnimation[] animations) {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LightsIOInputs inputs) {}

    public default void playAnimation(){}

    public default TimedAnimation getAnimation() {
        return TimedAnimation.DEFAULT;
    }

    public default void updateSim() {}
}
