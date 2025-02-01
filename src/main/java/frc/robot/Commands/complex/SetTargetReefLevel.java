package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;

public class SetTargetReefLevel extends Command {
  public SetTargetReefLevel(ElevatorStates targetLevel) {
    Elevator.getInstance().setState(targetLevel);
    Wrist.getInstance().setState(targetLevel, ClosedLoopSlot.kSlot0);
  }
}
