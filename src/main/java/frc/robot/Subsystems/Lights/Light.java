package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
//link to CANdle documentation: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
//need to set up advantage kit or sumthin, idk

public class Light extends SubsystemBase {
    private static Light instance;   
    private final LightsIO io; 
    public static Light getInstance() {
        if (instance == null) {
            throw new IllegalStateException("AlgaeRollers instance not set");
        }
        return instance;
    }

    public static void setInstance(LightsIO io) {
        instance = new Light(io);
    }

    public boolean command = false;
    private ArrayList<Animations> animationsList = new ArrayList<Animations>();
    //Honestly some of the best logic code I've ever written. // TODO - Document this later
    public Light(LightsIO io) {
        this.io = io;
    }

    int flag = 0;
    double start = 0;
    public void periodic() {
        if (animationsList.isEmpty()) {
            return;
        }
        io.playAnimation(start, flag);
    }

    public void setAnimation(Animations animation){
        io.setAnimation(animation);
    }

    public void setAnimation(Animations[] animations){
        io.setAnimation(animations);
    }
    
}
