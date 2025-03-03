package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

import org.littletonrobotics.junction.Logger;
//link to CANdle documentation: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
//need to set up advantage kit or sumthin, idk

public class Light extends SubsystemBase {
    private static Light instance;   
    private final LightsIO io; 
    int flag = 0;
    double start = 0;
    int targetTagID;
    public static Light getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Light instance not set");
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

    public void periodic() {
        switch (Swerve.getInstance().getReefTargetOrientation()) {
            case AB:
                targetTagID = Constants.isBlueAlliance ? 18 : 7;
            case CD:
                targetTagID = Constants.isBlueAlliance ? 17 : 8;
            case EF:
                targetTagID = Constants.isBlueAlliance ? 22 : 9;
            case GH:
                targetTagID = Constants.isBlueAlliance ? 21 : 10;
            case IJ:
                targetTagID = Constants.isBlueAlliance ? 20 : 11;
            case KL:
                targetTagID = Constants.isBlueAlliance ? 19 : 6;
            case NONE:
                targetTagID = -1;
        }
        if(AlignVision.getInstance().TagIsInView(targetTagID)){
            io.setAnimation(Animations.CANSEEREEFTAG);
        }else{
            io.setAnimation(Animations.CANNOTSEEREEFTAG);
        }
        
        io.playAnimation(start, flag);
    }
    
}
