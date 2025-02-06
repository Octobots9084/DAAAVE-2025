package frc.robot.Commands.ManualControl;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonConfig;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class ElevatorManualControl extends Command{
    DoubleSupplier vY;
    double height = -1;
    public ElevatorManualControl(DoubleSupplier vY) {
        this.vY = vY;
        this.addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute(){
        if (vY.getAsDouble() != 0)
        {
            if(height == -1)
                height = Elevator.getInstance().getPosition() + vY.getAsDouble();
            else
                height += vY.getAsDouble();
            Elevator.getInstance().manuelSetTargetPosistion(height);
        }
        
        
    }
}