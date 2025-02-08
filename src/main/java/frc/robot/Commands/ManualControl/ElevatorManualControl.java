
package frc.robot.Commands.ManualControl;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class ElevatorManualControl extends Command{
    DoubleSupplier vY;
    double height;
    public ElevatorManualControl(DoubleSupplier vY) {
        height = -1;
        this.vY = vY;
        this.addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute(){
        if (vY.getAsDouble() != 0)
        {
            if(height == -1)
                height = Elevator.getInstance().getPosition() + vY.getAsDouble()/3;
            else
                height += vY.getAsDouble()/3;
            ElevatorStates state = ElevatorStates.MANUAL;
            state.position = height;
            Elevator.getInstance().setState(state); 
        }
        
        
    }
}