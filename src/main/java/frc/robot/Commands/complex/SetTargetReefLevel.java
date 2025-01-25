package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ReefTargetLevel;
import frc.robot.Subsystems.Elevator.Elevator;

public class SetTargetReefLevel extends Command {
  public SetTargetReefLevel(ReefTargetLevel targetLevel) {
    Elevator.getInstance().setReefTargetLevel(targetLevel);
  }
}
