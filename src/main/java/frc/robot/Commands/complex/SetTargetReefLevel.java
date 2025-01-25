package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;

public class SetTargetReefLevel extends Command {
  public SetTargetReefLevel(ElevatorStates targetLevel) {
        Elevator.getInstance().setReefTargetLevel(targetLevel);
        Wrist.getInstance().setState(targetLevel);
    }
}
