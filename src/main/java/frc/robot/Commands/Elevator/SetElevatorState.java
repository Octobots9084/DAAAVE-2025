package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putString("state", targetState.toString());
    Elevator.getInstance().setState(targetState);
  }

  @Override
  public boolean isFinished () {
    return Elevator.getInstance().isAtState(0.1);//TODO: set actual tolerance
  }
}
