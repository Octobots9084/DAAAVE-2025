package frc.robot.Commands.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepCoral extends Command {

  @Override
  public void initialize() {
    Wrist.getInstance().setState(WristStates.PREP, ClosedLoopSlot.kSlot0); //TODO: change slot
  }

  @Override
  public boolean isFinished() {
    return Wrist.getInstance().isAtState(WristStates.PREP, 0.1); //TODO: change tolerance
  }
}
