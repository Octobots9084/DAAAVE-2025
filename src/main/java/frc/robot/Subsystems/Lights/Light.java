package frc.robot.Subsystems.Lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Subsystems.Elevator.*;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

import org.littletonrobotics.junction.Logger;
//link to CANdle documentation: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html
//need to set up advantage kit or sumthin, idk

public class Light extends SubsystemBase {
    private static Light instance;   
    private final LightsIO io; 
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

    public Light(LightsIO io) {
        this.io = io;
        io.getcandle().configBrightnessScalar(0.5);
    }

    public void candleOff(){
        io.getcandle().setLEDs(0,0,0);
    }

    public void lights() {
        ReefTargetOrientation orientation = Swerve.getInstance().getReefTargetOrientation();
        switch (orientation) {
            case AB:
                targetTagID = Constants.isBlueAlliance ? 18 : 7;
                break;
            case CD:
                targetTagID = Constants.isBlueAlliance ? 17 : 8;
                break;   
            case EF:
                targetTagID = Constants.isBlueAlliance ? 22 : 9;
                break;
            case GH:
                targetTagID = Constants.isBlueAlliance ? 21 : 10;
                break;
            case IJ:
                targetTagID = Constants.isBlueAlliance ? 20 : 11;
                break;
            case KL:
                targetTagID = Constants.isBlueAlliance ? 19 : 6;
                break;
            case NONE:
                targetTagID = -1;
            break;
        }
        if(Elevator.getInstance().getTargetState() == ElevatorStates.INTAKE && Wrist.getInstance().getState() == WristStates.INTAKE){
            candleOff();
            io.getcandle().setLEDs(148,0,211);
        }else{
            candleOff();
            if(AlignVision.getInstance().TagIsInView(targetTagID)){
                io.getcandle().setLEDs(0,255,0);
            }else{
                io.getcandle().setLEDs(255,0,0);

            }
        }

        // io.playAnimation();
    }
    
}
