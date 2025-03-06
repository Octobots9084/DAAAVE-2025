package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class SetElevatorState extends InstantCommand {
  private ElevatorStates targetState;

  public SetElevatorState(ElevatorStates targetState) {
    this.targetState = targetState;
  }

  @Override
  public void initialize() {
    Elevator.getInstance().setState(targetState);
  }

}
