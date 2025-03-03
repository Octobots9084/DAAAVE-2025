package frc.robot.Commands.Wrist;

import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;


public class SetWristState extends Command {
    private WristStates targetState;
    private ClosedLoopSlot slot;
    private Wrist wrist = Wrist.getInstance();
    private AlignVision vision = AlignVision.getInstance();

    public SetWristState(WristStates targetState, ClosedLoopSlot slot) {
        this.targetState = targetState;
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
        return wrist.isAtState(targetState, 0.03);
  }

}
