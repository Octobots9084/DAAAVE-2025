package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
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
  public boolean isFinished() {
    if((!((wrist.getPosition() > wrist.getHorizonAngle() && targetState.wristPosition < wrist.getHorizonAngle()) || (wrist.getPosition() < wrist.getHorizonAngle() && targetState.wristPosition > wrist.getHorizonAngle())) //is the wrist crossing the horizon
        && !(Math.abs(wrist.getPosition()-wrist.getHorizonAngle()) < 0.1 && Math.abs(targetState.wristPosition) < 0.1)) // is the target or current setpoint within the range
        || (vision.getLeftLidarDistance() > 0.1 || vision.getRightLidarDistance() > 0.1)) //is the robot away from the reef
        {
            Wrist.getInstance().setState(targetState, slot);
            return true;
        }
        return false;
    
  }
}
