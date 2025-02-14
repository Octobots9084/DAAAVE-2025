package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepReefPlacement extends InstantCommand {

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
        new SetWristStateTolerance(WristStates.PREP,
            0.05,
            ClosedLoopSlot.kSlot0),
        new SetElevatorStateTolerance(Elevator.getInstance().getTargetState(), 1.5),
        new SetWristStateTolerance(Elevator.getInstance().getTargetState(), 0.001, ClosedLoopSlot.kSlot0)));
  }
}
