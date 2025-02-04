package frc.robot.Subsystems.Lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants;

public enum Animations {
    DEFAULT(new StrobeAnimation(255, 255, 255, 0, 1, Constants.NUM_LEDS), 0),

    public Animation animation;
    public double time;
    private Animations(Animation animation, double time) {
        this.animation = animation;
        this.time = time;
    }

    
}
