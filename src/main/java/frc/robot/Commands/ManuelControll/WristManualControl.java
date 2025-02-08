package frc.robot.Commands.ManuelControll;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonConfig;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class WristManualControl extends Command{
    DoubleSupplier vX;
    double height = -1;
    public WristManualControl(DoubleSupplier vX) {
        this.vX = vX;
        this.addRequirements(Wrist.getInstance());
    }

    @Override
    public void execute(){
        if (vX.getAsDouble() != 0)
        {
            if(height == -1)
                height = Wrist.getInstance().getPosition() + vX.getAsDouble();
            else
                height += vX.getAsDouble()/200;
            WristStates state = WristStates.MANUAL;
            state.wristPosition = height;
            //TODO set position correctly (most likely the inital set point is off)
            //Wrist.getInstance().setState(state,ClosedLoopSlot.kSlot0); 
        }
        
        
    }
}