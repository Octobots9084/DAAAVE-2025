package frc.robot.Subsystems.Lights;

import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public enum TimedAnimation {
    DEFAULT(new StrobeAnimation(255, 255, 255, 0, 1, Constants.NUM_LEDS), 0),
    CANSEEREEFTAG(new StrobeAnimation(0,255,0,0,1,Constants.NUM_LEDS), 0),
    CANNOTSEEREEFTAG(new StrobeAnimation(255,0,0,0,1,Constants.NUM_LEDS),0);

    public Animation animation;
    public double time;
    private TimedAnimation(Animation animation, double time) {
        this.animation = animation;
        this.time = time;
    }
}
