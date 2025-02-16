package frc.robot.Commands.ReefSelection;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class SetTargetReefLevel extends Command {
  public SetTargetReefLevel(ElevatorStates targetLevel) {
    manager.level = targetLevel;
  }
}
