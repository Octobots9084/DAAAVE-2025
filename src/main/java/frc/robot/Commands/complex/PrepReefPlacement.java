package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepReefPlacement extends SequentialCommandGroup {
  public PrepReefPlacement(ElevatorStates elevatorState, WristStates wristState) {
    addCommands(new SetElevatorState(elevatorState), new SetWristState(wristState, ClosedLoopSlot.kSlot0));
  }
}
