package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class LightsIOSystem implements LightsIO {

    private CANdle candle;
    private ArrayList<Animations> animationsList = new ArrayList<Animations>();

    public LightsIOSystem() {
        this.candle = new CANdle(24);
    }

    public void setAnimation(Animations animations) {
        if (animations.time == 0) {
            configAnimation(animations.animation);
            return;
        }
        animationsList.add(animations);
    }

    public void setAnimation(Animations[] animations) {
        for (Animations animations2 : animations) {
            setAnimation(animations2);
        }
    }

    private Animation lastAnimation;
    private void configAnimation(Animation animation) {
        if (animation == lastAnimation) return;
        candle.animate(animation);
        lastAnimation = animation;
    }

    // int flag = 0;
    // double start = 0;
    // @Override
    // public void periodic() {
    //     if (animationsList.isEmpty()) {
    //         return;
    //     }
    //     switch (flag) {
    //         case 0:
    //             start = Timer.getFPGATimestamp();
    //             configAnimation(animationsList.get(0).animation);
    //             flag = 1;
    //             break;
    //         case 1:
    //             if (animationsList.get(0).time <= Timer.getFPGATimestamp() - start) {
    //                 animationsList.remove(0);
    //                 flag = 0;
    //                 configAnimation(Animations.DEFAULT.animation);
    //                 break;
    //             }
    //     }
    // }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        
    }
}
