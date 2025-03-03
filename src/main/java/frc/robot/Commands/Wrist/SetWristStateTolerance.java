package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;
import frc.robot.Subsystems.Vision.AlignVision;

public class SetWristStateTolerance extends Command {
  private WristStates targetState;
  private ElevatorStates targetElevatorState;
  private double tolerance;
  private Wrist wrist = Wrist.getInstance();
  private ClosedLoopSlot slot;
  private AlignVision vision = AlignVision.getInstance();

  public SetWristStateTolerance(WristStates targetState, double tolerance, ClosedLoopSlot slot) {
    this.targetState = targetState;
    this.tolerance = tolerance;
    this.slot = slot;
  }

  public SetWristStateTolerance(ElevatorStates elevatorState, double tolerance, ClosedLoopSlot slot) {
    if(elevatorState == ElevatorStates.LEVEL1)
        this.targetState = WristStates.L1;
    else if(elevatorState == ElevatorStates.LEVEL2)
        this.targetState = WristStates.L2;
    else if(elevatorState == ElevatorStates.LEVEL3)
        this.targetState = WristStates.L3;
    else if(elevatorState == ElevatorStates.LEVEL4)
        this.targetState = WristStates.L4;
    else
        throw new IllegalArgumentException("not l1, l2, l3, l4");

    this.tolerance = tolerance;
    this.slot = slot;
  }

  @Override
  public void execute() {
        boolean passingHorizon = (wrist.getPosition() > wrist.getHorizonAngle() && targetState.wristPosition < wrist.getHorizonAngle()) || (wrist.getPosition() < wrist.getHorizonAngle() && targetState.wristPosition > wrist.getHorizonAngle());
        boolean inhoizonZone = !(Math.abs(wrist.getPosition()-wrist.getHorizonAngle()) < 0.1 && Math.abs(targetState.wristPosition) < 0.1);
        boolean backedOffReef = (vision.getLeftLidarDistance() > 0.2 || vision.getRightLidarDistance() > 0.2);
        if(backedOffReef || (!passingHorizon || !inhoizonZone))
        {
            // only let the wrist got to intake when elvator is at L1
            if(targetState == WristStates.INTAKE && Elevator.getInstance().getPosition() < Elevator.BOT_CROSSBAR_POS) {
                Wrist.getInstance().setState(targetState, slot);
            }
            else {
                Wrist.getInstance().setState(targetState, slot);
            }
        }
    }

  @Override
  public boolean isFinished() {
    if (targetState == null) {
      return wrist.isAtState(targetElevatorState, tolerance);

    } else {
      return wrist.isAtState(targetState, tolerance);
    }
  }
}
