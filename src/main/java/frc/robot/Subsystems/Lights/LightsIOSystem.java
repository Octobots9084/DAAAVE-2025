package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.fasterxml.jackson.databind.deser.impl.PropertyValueBuffer;

import edu.wpi.first.wpilibj.Timer;

public class LightsIOSystem implements LightsIO {

    private CANdle candle;
    private TimedAnimation animation;

    public LightsIOSystem() {
        this.candle = new CANdle(24);
    }

    public void setAnimation(TimedAnimation animations) {
        this.animation = animations;
    }

    public void setAnimation(TimedAnimation[] animations) {
        for (TimedAnimation animations2 : animations) {
            setAnimation(animations2);
        }
    }

    @Override
    public TimedAnimation getAnimation(){
        return this.animation;
    }

    @Override
    public void playAnimation(){
        candle.animate(this.animation.animation);
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        
    }
}
