package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.fasterxml.jackson.databind.deser.impl.PropertyValueBuffer;

import edu.wpi.first.wpilibj.Timer;

public class LightsIOSystem implements LightsIO {

    public CANdle candle;
    private TimedAnimation animation;

    public LightsIOSystem() {
        this.candle = new CANdle(24, "KrakensBus");
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
        // ErrorCode error = candle.animate(this.animation.animation);
        this.candle.setLEDs(1, 1, 1);
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        
    }

    public CANdle getcandle() {
        return this.candle;
    }
}
